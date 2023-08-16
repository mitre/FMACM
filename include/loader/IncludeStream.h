// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "loader/Token.h"
#include "loader/RunFileArchiveDirector.h"
#include "loader/FilePath.h"
#include <assert.h>
#include <memory>

template <class PARENT>
class IncludeStream : public PARENT {
  public:
   IncludeStream(void)  // constructor
   {
      secondary = NULL;
      archive_director.reset();
   }  //----------------------------------------------------------------------------------------------

   ~IncludeStream(void)  // destructor
   {
      if (secondary != NULL) {
         delete secondary;
         secondary = NULL;
      }
   }  //----------------------------------------------------------------------------------------------

   inline bool open_file(const std::string &file_name)  // this over load is needed to track relative directory
                                                        // referance
   {
      FilePath temp = FilePath(file_name);
      local_path = temp.Pop();
      return PARENT::open_file(file_name);
   }  //----------------------------------------------------------------------------------------------

   inline FilePath get_file_path()  // TODO prototype this with a do nothing class  in the root class
   {
      return local_path;
   }  //----------------------------------------------------------------------------------------------

   Token get_next()  // get next token from include file- calls get next from token stream
   {
      Token out;
      Token temp;

      if (secondary == NULL) {
         out = get_next_from_primary();
      } else {
         temp = get_next_from_secondary();

         if (temp.get_Data().size() != 0) {
            return temp;
         } else {
            out = get_next_from_primary();

            // bring over formatting at end of secondary
            out.add_Format(temp.get_Format());

            out.add_Format('\n');  // put in a new line
         }
      }

      return out;
   }  //----------------------------------------------------------------------------------------------

   void set_Archive_Director(std::shared_ptr<RunFileArchiveDirector> new_ad) {
      assert(archive_director == NULL);
      archive_director = new_ad;
   }

   void set_Local_Path(FilePath fp) { local_path = fp; }

   //=====================================================================================
   // overloading indent management functions
   inline void tab_In() {
      if (secondary == NULL) {
         PARENT::tab_In();
      } else {
         secondary->tab_In();
      }
   }  //--------------------------------------------------------

   inline void tab_Out() {
      if (secondary == NULL) {
         PARENT::tab_Out();
      } else {
         secondary->tab_Out();
      }
   }  //--------------------------------------------------------

   inline int get_Tab() {
      if (secondary == NULL) {
         return PARENT::get_Tab();
      } else {
         return secondary->get_Tab();
      }
   }  //--------------------------------------------------------

   inline void set_Tab(int tab) {
      if (secondary == NULL) {
         PARENT::set_Tab();
      } else {
         secondary->set_Tab();
      }
   }  //--------------------------------------------------------

   inline std::string get_Space() {
      if (secondary == NULL) {
         return PARENT::get_Space();
      } else {
         return secondary->get_Space();
      }
   }
   //=====================================================================================
  private:
   Token get_next_from_primary() {
      Token out;
      out = PARENT::get_next();

      if (out.get_Data() == "#include") {
         out.add_Format(out.get_Data());
         out.set_data("");
         Token out1 = PARENT::get_next();
         out.add_Format(out1.get_Format());
         out.add_Format(out1.get_Data());
         std::string new_file_name = out1.get_Data();
         FilePath included_file = FilePath(new_file_name);

         // sets the local_path variable based on what the include file name is because
         // the include file name may be relative or absolute
         while (true) {
            // if((included_file.get_Full_Path().find("..")!=string::npos) || (included_file.get_Num_Dir()>1 &&
            // included_file.get_Disk()=="" && included_file.get_Type()!=""))
            bool has_mult_dirs = included_file.GetNumberOfDirectories() > 1;
            bool has_file_ext = included_file.GetType() != "";
#ifdef _MSC_VER
            bool has_dotdot = included_file.get_Full_Path().find("..") != string::npos;
            bool has_drive = included_file.get_Disk() != "";
            if (has_dotdot || (has_mult_dirs && !has_drive && has_file_ext)) {
               local_path = local_path.pop().cd(new_file_name);
               // cd_to_path = true;
               break;
            }

            // if(included_file.get_Num_Dir()>1 && included_file.get_Disk()!="" && included_file.get_Type()!="")
            if (has_mult_dirs && has_drive && has_file_ext) {

               local_path = included_file;
               abs_path_flag = true;
               break;
            }

            local_path = FilePath(local_path.get_Full_Path() + "\\" + included_file.get_Full_Path());
#else
            if (has_mult_dirs && has_file_ext) {

               local_path = included_file;
               break;
            }

            local_path = FilePath(local_path.GetFullPath() + "/" + included_file.GetFullPath());
#endif
            break;
         }

         new_file_name = local_path.GetFullPath();

         bool r = open_secondary(new_file_name);

         if (!r) {
            std::string e = "\n\nCould not open ";
            std::string f = new_file_name;
            std::string g = "\n";
            std::string h = e + f + g;
            PARENT::report_error(h);
         } else {
            tab_In();
            Token out2 = get_next();  // we call it recursively so we can deal with an empty include file
            out.add_Format("\n");
            out.add_Format(out2.get_Format());
            out.add_Data(out2.get_Data());
         }
      }

      return out;
   }  //----------------------------------------------------------------------------------------------

   Token get_next_from_secondary() {
      Token out;

      out = secondary->get_next();

      if (out.get_Data() == "") {
         secondary->write_Last_Token(out.get_Format());
         close_secondary();
      }

      return out;
   }  //----------------------------------------------------------------------------------------------

   bool open_secondary(const std::string &file_name) {
      assert(secondary == NULL);

      secondary = new IncludeStream<PARENT>;

      *secondary = *this;  // this should copy all the settings

      secondary->secondary =
            NULL;  // sets the original secondary to NULL so it doesn't point to itself and cause an infinite loop

      bool r = secondary->open_file(file_name);  // open the file we will be getting data from

      if (!r)  // we could not open the file
      {
         delete secondary;
         return false;
      } else {
         FilePath fp = FilePath(file_name);
         std::string name = fp.GetName();
         if (fp.GetType() != "") {
            name = name + "." + fp.GetType();
         }

         // if(!archive_director->is_new_File(fp)) // need to make an archive copy
         if (archive_director->is_new_File(fp))  // need to make an archive copy
         {
            std::string where_to_put_it = archive_director->get_Destination();
            FilePath loc = FilePath(where_to_put_it);
            FilePath to_open = loc.Push(FilePath(archive_director->get_New_Link_Name(file_name)));
            loc = loc.Push(FilePath(name));
            std::string comment = std::string("; This file is an archive copy of a file formerly called: \"") +
                                  file_name + std::string("\"\n");

            secondary->open_Archive(to_open.GetFullPath(), comment);
         }

         // Now we need to rewrite the include statement in the archive of the primary
         PARENT::rewrite_Last_in_Archive(file_name);

         return true;
      }

   }  //----------------------------------------------------------------------------------------------

   void close_secondary() {
      tab_Out();
      delete secondary;
      secondary = NULL;
   }  //----------------------------------------------------------------------------------------------

   IncludeStream<PARENT> *secondary;
   std::shared_ptr<RunFileArchiveDirector> archive_director;

   FilePath local_path;
};
