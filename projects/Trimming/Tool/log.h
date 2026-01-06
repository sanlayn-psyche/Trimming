#pragma once
#include <memory>
#include <string>
#include <fstream>
#include <mutex>
#include <iostream>
using std::cout;
using std::endl;
using std::string;
using std::shared_ptr;
using std::unique_ptr;

static int __debug_cnt = 0;
#define LF_LOG (*(LogDelegate::log()))
#define LF_LOG_APPEND(root) (*(LogDelegate::log(2, root)))
#define LF_LOG_OPEN(root) (*(LogDelegate::log(1, root)))
#define LF_LOG_CLOSE (*(LogDelegate::log(-1)))


inline int __debug_count()
{
	return ++__debug_cnt;
}

inline void __debug_hold(int couter)
{
	if (couter == __debug_cnt)
	{
		double xxx = 1.0;
	}
}

inline void __debug_break(bool flag)
{
	if (flag)
	{
		double xxx = 1.0;
	}
	
}

class LogDelegate
{
public:
	LogDelegate(const LogDelegate&) = delete;
	LogDelegate(LogDelegate&&) = delete;

	LogDelegate& operator=(const LogDelegate&) = delete;
	LogDelegate& operator=(LogDelegate&&) = delete;

	// -1 close, 0 follow, 1 open, 2 append
	static unique_ptr<std::ofstream>& log(int mode = 0,string root = "") noexcept
	{
		static LogDelegate inst;
		std::lock_guard<std::mutex> guard(inst.m_mutex);

		switch (mode)
		{
		case -1:
			inst.ofs->close();
			inst.ofs_a->close();
			inst.m_mode = -1;
			break;

		case 1:
			if (!root.empty())
			{
				inst.ofs->close();
				inst.ofs->open(root);
			}
			inst.m_mode = (inst.ofs->is_open() == true) ? 1 : -1;
			break;

		case 2:
			if (!root.empty())
			{
				inst.ofs_a->close();
				inst.ofs_a->open(root,std::ios_base::app);
			}
			inst.m_mode = (inst.ofs_a->is_open() == true) ? 2 : -1;
			break;

		default:
			if (inst.m_mode == 2 && !inst.ofs_a->is_open())
			{
				inst.m_mode = 1;
			}
			if (inst.m_mode > 0 && inst.ofs->is_open())
			{
				inst.m_mode = 1;
			}
			else
			{
				inst.m_mode = -1;
			}
			break;
		}

	
		switch (inst.m_mode)
		{
		case 1:
			return inst.ofs;
		case 2:
			return inst.ofs_a;
		default:
			return inst.os;
		}
	}



private:
	
	unique_ptr<std::ofstream> ofs;
	unique_ptr<std::ofstream> os;
	unique_ptr<std::ofstream> ofs_a;
	int m_mode;
	std::mutex m_mutex;

	LogDelegate():
		m_mode(-1)
	{
		ofs = std::make_unique<std::ofstream>();
		os = std::make_unique<std::ofstream>();
		ofs_a = std::make_unique<std::ofstream>();
		os->basic_ios<char>::rdbuf(std::cout.rdbuf());
	}

	~LogDelegate()
	{
		os->clear();
		ofs->close();
		ofs_a->close();
	}


};