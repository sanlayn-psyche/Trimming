#pragma once

#include <exception>
#include <string>
#include <iostream>

#define LF_EXCEPTION

class lf_exception: public std::exception
{
protected:
	int m_errorCode{ 0 };
	const void* m_suspect{ nullptr };
	std::string m_errorInfo;
	mutable std::string m_outputs;
	mutable int m_patchid{-1};

protected:

	virtual const void act_generate_info() const
	{
		m_outputs = "Error# " + std::to_string(m_errorCode) + ". ";
	}


public:
	lf_exception() {};
	void set_patchid(int id) const
	{
		m_patchid = id;
	}
	const char* what() const noexcept override
	{
		act_generate_info();
		return m_outputs.c_str();
	}
	virtual ~lf_exception() = default;
#ifdef _DEBUG 
	virtual void act_output() const = 0;
#endif // _DEBUG

};


class lf_exception_undefined : public lf_exception
{
public:
	lf_exception_undefined(const char* info = "Unknown problem. ")
	{
		m_errorInfo = info;
		m_errorCode = 0;
	}
#ifdef _DEBUG
	void act_output() const override {}
#endif // _DEBUG

private:
	virtual const void act_generate_info() const
	{
		lf_exception::act_generate_info();
		m_outputs += m_errorInfo;
	}
};

class lf_exception_node : public lf_exception
{
public:
	lf_exception_node() {};
	lf_exception_node(const void* node, const char* info = "Unknown problem. ")
	{
		m_errorInfo = info;
		m_errorCode = 1;
		m_suspect = node;
	}
#ifdef _DEBUG
	void act_output() const override;
#endif // _DEBUG
	virtual const void act_generate_info() const;
};

class lf_exception_curveset : public lf_exception
{
public:
	lf_exception_curveset(const void* curveset, const char* info = "Unknown problem. ")
	{
		m_errorInfo = info;
		m_errorCode = 2;
		m_suspect = curveset;
	}
#ifdef _DEBUG
	void act_output() const override;
#endif // _DEBUG
	virtual const void act_generate_info() const;
};

class lf_exception_subcurves : public lf_exception
{
public:
	lf_exception_subcurves(std::initializer_list<const void*> &&curves, const char* info = "Unknown problem. ")
	{
		m_errorInfo = info;
		m_errorCode = 3;
		for (auto x: curves)
		{
			m_curves.push_back(x);
		}
	}
#ifdef _DEBUG
	void act_output() const override;
#endif // _DEBUG
	virtual const void act_generate_info() const;
private:
	std::vector<const void*> m_curves;
};

class lf_exception_cut : public lf_exception_node
{
public:
	lf_exception_cut(const void* node, bool ifparent, const char* info = "Unexcepted cut detected. ")
	{
		m_errorInfo = info;
		m_errorCode = 4;
		m_suspect = node;
		m_if_parent = ifparent;
	}
	~lf_exception_cut() = default;
#ifdef _DEBUG
	void act_output() const override;
#endif // _DEBUG
	virtual const void act_generate_info() const;

private:
	bool m_if_parent{false};
};
