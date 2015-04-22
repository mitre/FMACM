// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once
#include <string>
#include <set>
#include <list>
#include <iostream>
#include <map>
#include <assert.h>

#include <Angle.h>
#include <Speed.h>
#include "CustomUnits.h"

#include "DecodedStream.h"
#include "LoaderLink.h"

#include "NativeLoaderLink.h"
#include "LoadableLoaderLink.h"
#include "NamedElementLoaderLink.h"
#include "ListLoaderLink.h"
#include "LoaderSupport.h"
#include "ScalarLoaderLink.h"
#include "LoadableLoaderLinkWithBrackets.h"

static const LoaderDeprecatedMetaInfo defMetaInfo = {false,""};

class Loadable: public LoaderSupport
{
public:
	Loadable(void);
	Loadable(const Loadable &in);
	virtual ~Loadable(void);
	virtual void operator=(const Loadable &in);

	virtual bool load(DecodedStream *input) = 0; // all children of this class must implement this function it will return true if it can load

	// for loading sequences of things that are not named

	template<class TYPE>
	bool load_datum(TYPE &datum) // load one var
	{
		bool out = stream->get_datum(datum);

		return out;
	}

	bool loadAngleDegrees(Units::Angle &angle);
	bool loadLengthFeet(Units::Length &length);
	bool loadSpeedKnots(Units::Speed &speed);
	bool loadAccelerationKnotsPerSecond(Units::Acceleration &acceleration);

	//========================================================================
	// for loading named vars ------------------------------------------------------
	
	/**
	 * Register an optional tag to load.
	 *
	 * @param name
	 * @param var_address
	 */
	template<class TYPE>
	void register_var(std::string name, TYPE *var_address)
	{
		register_var(name, var_address, false, defMetaInfo);
	}

	/**
	 * Use this to declare a variable and to control its required flag. By definition, a required variable is NOT deprecated.
	 *
	 * @param name
	 * @param var_address
	 * @param required
	 * @param deprecated
	 */
	template<class TYPE>
	void register_var(std::string name, TYPE *var_address, bool isRequired)
	{
		register_var(name, var_address, isRequired, defMetaInfo);
	}

	/**
	 * Use this to declare a variable and to control its deprecated flag. The required flag will be set to false.
	 *
	 * @param name
	 * @param var_address
	 */
	template<class TYPE>
	void register_var(std::string name, TYPE *var_address, LoaderDeprecatedMetaInfo info)
	{
		register_var(name, var_address, false, info);
	}

	// for loading lists ---------------------------------------------------------
	// in the run file the named list appears once and is proceeded by the name of the list

	/**
	 * Register an optional LOADABLE_TYPE named list.
	 * @param name
	 * @param target
	 */
	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function
	void register_named_list_item(std::string name, std::list<LOADABLE_TYPE> *target)
	{
		register_named_list_item(name, target, false, defMetaInfo);
	}

	/**
	 * Register an optional LOADABLE_TYPE named list with required information.
	 *
	 * @param name
	 * @param target
	 * @param isRequired
	 */
	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function
	void register_named_list_item(std::string name, std::list<LOADABLE_TYPE> *target, bool isRequired)
	{
		register_named_list_item(name, target, isRequired, defMetaInfo);
	}

	/**
	 * Register an optional LOADABLE_TYPE named list with deprecated information.
	 *
	 * @param name
	 * @param target
	 * @param info
	 */
	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function
	void register_named_list_item(std::string name, std::list<LOADABLE_TYPE> *target, LoaderDeprecatedMetaInfo info)
	{
		register_named_list_item(name, target, false, info);
	}

	//---------------------------------------------------------------------------------------------------

	/**
	 * Register an optional LOADABLE_TYPE list. LOADABLE_TYPE Must be a child of loadable or provide the necessary load function.
	 *
	 * @param name
	 * @param target
	 */
	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function 
	void register_named_list(const std::string &name, std::list<LOADABLE_TYPE> *target)
	{
		register_named_list(name, target, false, defMetaInfo);
	}

	/**
	 * Register a LOADABLE_TYPE list with an specific required setting. LOADABLE_TYPE Must be a child of loadable or provide the necessary load function.
	 *
	 * @param name
	 * @param target
	 * @param isRequired
	 */
	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function
	void register_named_list(const std::string &name, std::list<LOADABLE_TYPE> *target, bool isRequired)
	{
		register_named_list(name, target, isRequired, defMetaInfo);
	}

	/**
	 * Register an optional LOADABLE_TYPE list with an specific LoaderDeprecatedMetaInfo setting. LOADABLE_TYPE Must be a child of loadable or provide the necessary load function.
	 *
	 * @param name
	 * @param target
	 * @param info
	 */
	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function
	void register_named_list(const std::string &name, std::list<LOADABLE_TYPE> *target, LoaderDeprecatedMetaInfo info)
	{
		register_named_list(name, target, false, info);
	}

	//void register_named_list(string heading, Loadable_List *list); // this is used if all the list items are in a single block
	//----------------------------------------------------------------------------------------------------------------------------

	/**
	 * Register an optional inline loadable tag. This is useful for multiple parameters on a single line with a custom Loadable.
	 *
	 * @param name
	 * @param target
	 */
	template <class LOADABLE_TYPE> 
	void register_loadable(const std::string &name, LOADABLE_TYPE *target)
	{
		register_loadable(name, target, false, defMetaInfo);
	}

	/**
	 * Register an inline loadable tag with specific required properties.
	 *
	 * @param name
	 * @param target
	 * @param isRequired
	 */
	template <class LOADABLE_TYPE>
	void register_loadable(const std::string &name, LOADABLE_TYPE *target, bool isRequired)
	{
		register_loadable(name, target, isRequired, defMetaInfo);
	}

	/**
	 * Register an optional inline loadable tag with specific deprecated properties.
	 *
	 * @param name
	 * @param target
	 * @param info
	 */
	template <class LOADABLE_TYPE>
	void register_loadable(const std::string &name, LOADABLE_TYPE *target, LoaderDeprecatedMetaInfo info)
	{
		register_loadable(name, target, false, info);
	}

	//-------------------------------------------------------------------------------------------------
	
	/**
	 * This will register the bracket as optional and not deprecated.
	 *
	 * @param name bracket name
	 * @param target pointer to storage variable
	 */
	template <class LOADABLE_TYPE> 
	void register_loadable_with_brackets(const std::string &name, LOADABLE_TYPE *target)
	{
		register_loadable_with_brackets(name, target, false, defMetaInfo);
	}

	/**
	 * Register a loadable tag with and set the required field.
	 *
	 * @param name
	 * @param target
	 * @param isRequired
	 */
	template <class LOADABLE_TYPE>
	void register_loadable_with_brackets(const std::string &name, LOADABLE_TYPE *target, bool isRequired)
	{
		register_loadable_with_brackets(name, target, isRequired, defMetaInfo);
	}

	/**
	 * Register an optional loadable tag and set the deprecated information.
	 *
	 * @param name
	 * @param target
	 * @param info
	 */
	template <class LOADABLE_TYPE>
	void register_loadable_with_brackets(const std::string &name, LOADABLE_TYPE *target, LoaderDeprecatedMetaInfo info)
	{
		register_loadable_with_brackets(name, target, false, info);
	}

	/**
	 * Call this after you finish registering things at it will load all the things it could and return false if it fails
	 *
	 * @return
	 * @see loaded_successfully()
	 */
	bool complete();

	void dump(); // dump them to cout

	void report_error(string error_message);

	void report_warning(string warning_message);

	/**
	 * returns true if the call to complete() went well
	 *
	 * @return
	 */
	bool loaded_successfully();

protected://----------------------------------------------------------------------------------------------------------------------------

	void set_stream(DecodedStream *input) // This should be called in the loader function of the child 
	{
		stream = input;
	}

	void cleanup();
	bool test_load();

private://----------------------------------------------------------------

	DecodedStream *stream;
	bool was_load_successful;
	std::map<std::string, LoaderLink*> lookup_table;

	#ifdef _DEBUG
		
		template <class P>
		void check_address(P* a)
		{
			assert(set1.find((void*)a) == set1.end()); //if assertion fails, it is because someone tried to register same address more than once
			set1.insert((void*)a);
		}

		set<void*> set1;

	#endif

	template<class TYPE>
	void register_var(std::string name, TYPE *var_address, bool isRequired, LoaderDeprecatedMetaInfo depInfo)
	{
		std::string varnameclean = clean_token(name);

		#ifdef _DEBUG
				std::cout << "In register_var: " << *var_address << std::endl; // returns the preset value if one exists
				check_address(var_address);
		#endif

		assert(lookup_table.find(varnameclean) == lookup_table.end()); // The user caller to register the variable name twice

		LoaderLink *new_link = new NativeLoaderLink<TYPE>(var_address);
		new_link->set_must_load(isRequired); // set required flag
		new_link->set_deprecated_info(depInfo);

		lookup_table[varnameclean] = new_link;
	}

	template <class LOADABLE_TYPE>
	void register_loadable_with_brackets(const std::string &name, LOADABLE_TYPE *target, bool isRequired, LoaderDeprecatedMetaInfo depInfo)
	{
		std::string varnameclean = clean_token(name);

		#ifdef _DEBUG
			check_address(target);
		#endif

		assert(lookup_table.find(varnameclean) == lookup_table.end()); // The user caller to register the variable name twice

		LoaderLink *new_link = new LoadableLoaderLinkWithBrackets<LOADABLE_TYPE>(target);
		new_link->set_must_load(isRequired);
		new_link->set_deprecated_info(depInfo);

		lookup_table[varnameclean] = new_link;
	}

	template <class LOADABLE_TYPE>
	void register_loadable(const std::string &name, LOADABLE_TYPE *target, bool isRequired, LoaderDeprecatedMetaInfo info)
	{
		std::string varnameclean = clean_token(name);

		#ifdef _DEBUG
				check_address(target);
		#endif
		assert(lookup_table.find(varnameclean) == lookup_table.end()); // The user caller to register the variable name twice

		LoaderLink *new_link = new LoadableLoaderLink<LOADABLE_TYPE>(target);
		new_link->set_must_load(isRequired);
		new_link->set_deprecated_info(info);

		lookup_table[varnameclean] = new_link;
	}

	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function
	void register_named_list(const std::string &name, std::list<LOADABLE_TYPE> *target, bool isRequired, LoaderDeprecatedMetaInfo info)
	{
		std::string varnameclean = clean_token(name);

		#ifdef _DEBUG
				check_address(target);
		#endif
		assert(lookup_table.find(varnameclean) == lookup_table.end()); // The user caller to register the variable name twice

		LoaderLink *new_link = new ListLoaderLink<LOADABLE_TYPE>(target);
		new_link->set_must_load(isRequired);
		new_link->set_deprecated_info(info);

		lookup_table[varnameclean] = new_link;
	}

	template <class LOADABLE_TYPE> // LOADABLE_TYPE Must be a child of loadable or provide the necessary load function
	void register_named_list_item(std::string name, std::list<LOADABLE_TYPE> *target, bool isRequired, LoaderDeprecatedMetaInfo info)
	{
		std::string varnameclean = clean_token(name);

		#ifdef _DEBUG
				check_address(target);
		#endif

		assert(lookup_table.find(varnameclean) == lookup_table.end()); // The user caller to register the variable name twice

		LoaderLink *new_link = new NamedElementLoaderLink<LOADABLE_TYPE>(target);
		new_link->set_must_load(isRequired);
		new_link->set_deprecated_info(info);

		lookup_table[varnameclean] = new_link;
	}

};
