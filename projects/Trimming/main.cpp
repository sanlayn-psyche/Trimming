#include <iostream>
#include "TrimManager.h"
#include "Patch.h"
#include <atomic>
#include <thread>

const std::string ProjectPath {RootPath};

int main(int argc, char** argv)
{
	TrimManager* tm_ptr{nullptr};
	std::string config_path {ProjectPath + "/config.json"};
	tm_ptr = new TrimManager( config_path.c_str());

#ifdef _DEBUG
	tm_ptr->init_resolve();
	Patch pc(tm_ptr->m_patch_prop);
	tm_ptr->act_resolve(tm_ptr->m_startId, &pc);

#else
	if (argc == 2)
	{
		if (string(argv[1]) == string("-h"))
		{
			std::cout << std::endl;
			std::cout << "Trimming help:" << std::endl;
			std::cout << "-rs: to resolve rare geometry data and generate data for rendering per patch." << std::endl;
			std::cout << "-mg: to merge data for rendering." << std::endl;
			return -1;
		}
		else if (string(argv[1]) == string("-rs"))
		{
			std::cout << std::endl;
			std::cout << "Model processing..." << std::endl;
			tm_ptr->act_resolveModel();
		}
		else if (string(argv[1]) == string("-mg"))
		{
			std::cout << std::endl;
			std::cout << "File merging..." << std::endl;
			tm_ptr->act_combine();
		}
		else
		{
			std::cout << std::endl;
			std::cout << "Error: undefined input, please try again or use -h to get help." << std::endl;
			return -1;
		}
}
	else if (argc == 1)
	{
		tm_ptr->run();
	}
	else
	{
		std::cout << "Error: unexpected input! use -h to get help" << std::endl;
		return -1;
	}
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

#endif
	

	delete tm_ptr;
	return 0;
}