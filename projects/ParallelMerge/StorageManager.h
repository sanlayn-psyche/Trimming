#pragma once

/**
 * @file StorageManager.h
 * @brief 高性能存储工具类
 * 
 * 提供两种存储抽象：
 * - RecordFile: 定长记录表，使用内存映射
 * - DataHeap:   变长数据堆，使用原子占坑 + 异步写入
 * 
 * 当前实现针对 Windows 平台优化
 */

#include <atomic>
#include <cstdint>
#include <string>
#include <mutex>
#include <stdexcept>
#include <algorithm>
#include <cstring>

#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <Windows.h>
#else
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <unistd.h>
#endif

namespace parallel_merge {

/**
 * @brief 定长记录文件（内存映射）
 * 
 * 适用于固定大小的 Record 表（如 Offset 表）
 * 通过内存映射实现高效的随机访问写入
 */
class RecordFile {
public:
    RecordFile() = default;
    
    ~RecordFile() {
        Close();
    }
    
    // 禁用拷贝
    RecordFile(const RecordFile&) = delete;
    RecordFile& operator=(const RecordFile&) = delete;
    
    // 允许移动
    RecordFile(RecordFile&& other) noexcept {
        *this = std::move(other);
    }
    
    RecordFile& operator=(RecordFile&& other) noexcept {
        if (this != &other) {
            Close();
            mappedView_ = other.mappedView_;
            fileSize_ = other.fileSize_;
            recordSize_ = other.recordSize_;
            recordCount_ = other.recordCount_;
#ifdef _WIN32
            fileHandle_ = other.fileHandle_;
            mappingHandle_ = other.mappingHandle_;
            other.fileHandle_ = INVALID_HANDLE_VALUE;
            other.mappingHandle_ = nullptr;
#else
            fd_ = other.fd_;
            other.fd_ = -1;
#endif
            other.mappedView_ = nullptr;
        }
        return *this;
    }

    /**
     * @brief 创建并映射记录文件
     * 
     * @param path        文件路径
     * @param recordSize  单条记录的字节大小
     * @param recordCount 记录数量
     * @return 成功返回 true
     */
    bool Create(const std::string& path, size_t recordSize, size_t recordCount) {
        Close();
        
        recordSize_ = recordSize;
        recordCount_ = recordCount;
        fileSize_ = recordSize * recordCount;
        
        if (fileSize_ == 0) {
            return false;
        }
        
#ifdef _WIN32
        // Windows: CreateFile + CreateFileMapping + MapViewOfFile
        fileHandle_ = CreateFileA(
            path.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,  // 独占访问
            nullptr,
            CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL,
            nullptr
        );
        
        if (fileHandle_ == INVALID_HANDLE_VALUE) {
            return false;
        }
        
        // 设置文件大小
        LARGE_INTEGER size;
        size.QuadPart = static_cast<LONGLONG>(fileSize_);
        if (!SetFilePointerEx(fileHandle_, size, nullptr, FILE_BEGIN) ||
            !SetEndOfFile(fileHandle_)) {
            CloseHandle(fileHandle_);
            fileHandle_ = INVALID_HANDLE_VALUE;
            return false;
        }
        
        // 创建文件映射
        mappingHandle_ = CreateFileMappingA(
            fileHandle_,
            nullptr,
            PAGE_READWRITE,
            static_cast<DWORD>(fileSize_ >> 32),
            static_cast<DWORD>(fileSize_ & 0xFFFFFFFF),
            nullptr
        );
        
        if (!mappingHandle_) {
            CloseHandle(fileHandle_);
            fileHandle_ = INVALID_HANDLE_VALUE;
            return false;
        }
        
        // 映射视图
        mappedView_ = MapViewOfFile(
            mappingHandle_,
            FILE_MAP_ALL_ACCESS,
            0, 0, 0  // 映射整个文件
        );
        
        if (!mappedView_) {
            CloseHandle(mappingHandle_);
            CloseHandle(fileHandle_);
            mappingHandle_ = nullptr;
            fileHandle_ = INVALID_HANDLE_VALUE;
            return false;
        }
#else
        // POSIX: open + ftruncate + mmap
        fd_ = open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd_ < 0) {
            return false;
        }
        
        if (ftruncate(fd_, static_cast<off_t>(fileSize_)) != 0) {
            close(fd_);
            fd_ = -1;
            return false;
        }
        
        mappedView_ = mmap(nullptr, fileSize_, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd_, 0);
        if (mappedView_ == MAP_FAILED) {
            mappedView_ = nullptr;
            close(fd_);
            fd_ = -1;
            return false;
        }
#endif
        
        return true;
    }
    
    /**
     * @brief 写入指定槽位的记录
     * 
     * @param slot 槽位索引 [0, recordCount)
     * @param data 数据指针
     * @param size 数据大小（应等于 recordSize）
     */
    void WriteRecord(size_t slot, const void* data, size_t size) {
        if (!mappedView_ || slot >= recordCount_ || size > recordSize_) {
            return;
        }
        
        char* dest = static_cast<char*>(mappedView_) + slot * recordSize_;
        std::memcpy(dest, data, size);
    }
    
    /**
     * @brief 获取指定槽位的指针（用于直接写入）
     */
    [[nodiscard]] void* GetSlotPtr(size_t slot) {
        if (!mappedView_ || slot >= recordCount_) {
            return nullptr;
        }
        return static_cast<char*>(mappedView_) + slot * recordSize_;
    }
    
    /**
     * @brief 关闭文件并释放资源
     */
    void Close() {
#ifdef _WIN32
        if (mappedView_) {
            UnmapViewOfFile(mappedView_);
            mappedView_ = nullptr;
        }
        if (mappingHandle_) {
            CloseHandle(mappingHandle_);
            mappingHandle_ = nullptr;
        }
        if (fileHandle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(fileHandle_);
            fileHandle_ = INVALID_HANDLE_VALUE;
        }
#else
        if (mappedView_) {
            munmap(mappedView_, fileSize_);
            mappedView_ = nullptr;
        }
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
#endif
        fileSize_ = 0;
    }
    
    /**
     * @brief 刷新到磁盘
     */
    void Flush() {
#ifdef _WIN32
        if (mappedView_) {
            FlushViewOfFile(mappedView_, 0);
        }
        if (fileHandle_ != INVALID_HANDLE_VALUE) {
            FlushFileBuffers(fileHandle_);
        }
#else
        if (mappedView_) {
            msync(mappedView_, fileSize_, MS_SYNC);
        }
#endif
    }
    
    [[nodiscard]] bool IsOpen() const { return mappedView_ != nullptr; }
    [[nodiscard]] size_t GetFileSize() const { return fileSize_; }
    [[nodiscard]] size_t GetRecordSize() const { return recordSize_; }
    [[nodiscard]] size_t GetRecordCount() const { return recordCount_; }

private:
    void* mappedView_{nullptr};
    size_t fileSize_{0};
    size_t recordSize_{0};
    size_t recordCount_{0};
    
#ifdef _WIN32
    HANDLE fileHandle_{INVALID_HANDLE_VALUE};
    HANDLE mappingHandle_{nullptr};
#else
    int fd_{-1};
#endif
};


/**
 * @brief 变长数据堆
 * 
 * 适用于变长二进制数据的并行写入：
 * - 原子占坑（Space Reservation）
 * - 无锁并行写入
 * - 自动扩容（1.5x-2x 策略）
 * - 最终收缩（Shrink to fit）
 */
class DataHeap {
public:
    /// 默认初始文件大小：16 MB
    static constexpr size_t kDefaultInitialSize = 16 * 1024 * 1024;
    
    /// 扩容因子
    static constexpr double kGrowthFactor = 1.5;

    DataHeap() = default;
    
    ~DataHeap() {
        Close();
    }
    
    // 禁用拷贝
    DataHeap(const DataHeap&) = delete;
    DataHeap& operator=(const DataHeap&) = delete;

    /**
     * @brief 创建数据堆文件
     * 
     * @param path        文件路径
     * @param initialSize 初始文件大小（字节）
     */
    bool Create(const std::string& path, size_t initialSize = kDefaultInitialSize) {
        Close();
        
        path_ = path;
        fileSize_.store(initialSize, std::memory_order_relaxed);
        dataPointer_.store(0, std::memory_order_relaxed);
        
#ifdef _WIN32
        fileHandle_ = CreateFileA(
            path.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            CREATE_ALWAYS,
            FILE_FLAG_OVERLAPPED,  // 异步 I/O
            nullptr
        );
        
        if (fileHandle_ == INVALID_HANDLE_VALUE) {
            return false;
        }
        
        // 设置初始大小
        LARGE_INTEGER size;
        size.QuadPart = static_cast<LONGLONG>(initialSize);
        if (!SetFilePointerEx(fileHandle_, size, nullptr, FILE_BEGIN) ||
            !SetEndOfFile(fileHandle_)) {
            CloseHandle(fileHandle_);
            fileHandle_ = INVALID_HANDLE_VALUE;
            return false;
        }
#else
        fd_ = open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd_ < 0) {
            return false;
        }
        
        if (ftruncate(fd_, static_cast<off_t>(initialSize)) != 0) {
            close(fd_);
            fd_ = -1;
            return false;
        }
#endif
        
        return true;
    }
    
    /**
     * @brief 预留写入空间（原子操作）
     * 
     * @param length 需要的字节数
     * @return 分配的起始偏移量
     */
    size_t Reserve(size_t length) {
        size_t startOffset = dataPointer_.fetch_add(length, std::memory_order_acq_rel);
        size_t endOffset = startOffset + length;
        
        // 检查是否需要扩容
        size_t currentSize = fileSize_.load(std::memory_order_acquire);
        if (endOffset > currentSize) {
            EnsureCapacity(endOffset);
        }
        
        return startOffset;
    }
    
    /**
     * @brief 写入数据到指定偏移
     * 
     * @param offset 写入偏移量（由 Reserve 返回）
     * @param data   数据指针
     * @param length 数据长度
     * @return 成功返回 true
     */
    bool Write(size_t offset, const void* data, size_t length) {
#ifdef _WIN32
        OVERLAPPED overlapped = {};
        overlapped.Offset = static_cast<DWORD>(offset & 0xFFFFFFFF);
        overlapped.OffsetHigh = static_cast<DWORD>(offset >> 32);
        overlapped.hEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
        
        if (!overlapped.hEvent) {
            return false;
        }
        
        DWORD bytesWritten = 0;
        BOOL result = WriteFile(
            fileHandle_,
            data,
            static_cast<DWORD>(length),
            &bytesWritten,
            &overlapped
        );
        
        if (!result && GetLastError() == ERROR_IO_PENDING) {
            // 等待异步完成
            result = GetOverlappedResult(fileHandle_, &overlapped, &bytesWritten, TRUE);
        }
        
        CloseHandle(overlapped.hEvent);
        return result && bytesWritten == length;
#else
        ssize_t written = pwrite(fd_, data, length, static_cast<off_t>(offset));
        return written == static_cast<ssize_t>(length);
#endif
    }
    
    /**
     * @brief 确保文件容量足够
     * 
     * 使用 1.5x 扩张策略
     */
    void EnsureCapacity(size_t requiredSize) {
        size_t currentSize = fileSize_.load(std::memory_order_acquire);
        if (requiredSize <= currentSize) {
            return;
        }
        
        // 争抢扩容权
        std::lock_guard<std::mutex> lock(resizeMutex_);
        
        // Double-check
        currentSize = fileSize_.load(std::memory_order_acquire);
        if (requiredSize <= currentSize) {
            return;
        }
        
        // 计算新大小（1.5x 或至少满足需求）
        size_t newSize = static_cast<size_t>(currentSize * kGrowthFactor);
        newSize = (std::max)(newSize, requiredSize);
        
        Resize(newSize);
    }
    
    /**
     * @brief 调整文件大小
     */
    void Resize(size_t newSize) {
#ifdef _WIN32
        LARGE_INTEGER size;
        size.QuadPart = static_cast<LONGLONG>(newSize);
        SetFilePointerEx(fileHandle_, size, nullptr, FILE_BEGIN);
        SetEndOfFile(fileHandle_);
#else
        ftruncate(fd_, static_cast<off_t>(newSize));
#endif
        fileSize_.store(newSize, std::memory_order_release);
    }
    
    /**
     * @brief 收缩文件到实际使用大小
     */
    void ShrinkToFit() {
        size_t actualSize = dataPointer_.load(std::memory_order_acquire);
        if (actualSize > 0) {
            Resize(actualSize);
        }
    }
    
    /**
     * @brief 关闭文件
     */
    void Close() {
#ifdef _WIN32
        if (fileHandle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(fileHandle_);
            fileHandle_ = INVALID_HANDLE_VALUE;
        }
#else
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
#endif
    }
    
    [[nodiscard]] size_t GetDataPointer() const {
        return dataPointer_.load(std::memory_order_acquire);
    }
    
    [[nodiscard]] size_t GetFileSize() const {
        return fileSize_.load(std::memory_order_acquire);
    }

private:
    std::string path_;
    std::atomic<size_t> dataPointer_{0};
    std::atomic<size_t> fileSize_{0};
    std::mutex resizeMutex_;
    
#ifdef _WIN32
    HANDLE fileHandle_{INVALID_HANDLE_VALUE};
#else
    int fd_{-1};
#endif
};

} // namespace parallel_merge
