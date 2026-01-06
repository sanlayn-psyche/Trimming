#include <iostream>
#include "TrimManager.h"
#include "Patch.h"
#include <atomic>
#include <thread>

#ifdef _DEBUG
#include "vld.h"
#endif

int main(int argc, char** argv)
{

	TrimManager* tm_ptr{nullptr};
	tm_ptr = new TrimManager();
	tm_ptr->run();


	//if (argc == 2)
	//{
	//	if (string(argv[1]) == string("-h"))
	//	{
	//		std::cout << std::endl;
	//		std::cout << "Trimming help:" << std::endl;
	//		std::cout << "-rs: to resolve rare geometry data and generate data for rendering per patch." << std::endl;
	//		std::cout << "-mg: to merge data for rendering." << std::endl;
	//	}
	//	else
	//	{
	//		std::cout << std::endl;
	//		std::cout << "Error: undefined input, please try again or use -h to get help." << std::endl;
	//	}

	//}
	//else if (argc == 3)
	//{
	//	if (string(argv[1]) == string("-rs"))
	//	{
	//		std::cout << std::endl;
	//		std::cout << "Model processing..." << std::endl;
	//		tm_ptr = new TrimManager();
	//		tm_ptr->act_resolveModel();
	//	}
	//	if (string(argv[1]) == string("-mg"))
	//	{
	//		std::cout << std::endl;
	//		std::cout << "File merging..." << std::endl;
	//		tm_ptr = new TrimManager();
	//		tm_ptr->act_combineFile();
	//	}
	//}
	//else
	//{
	//	std::cout << "Error: unexpected input! use -h to get help" << std::endl;
	//}
	extern std::atomic<unsigned> g_threadCnt;
	while (true)
	{
		if (0 < g_threadCnt.load())
		{
			std::this_thread::yield();
			continue;
		}
		break;
	}

	delete tm_ptr;
	return 0;
}