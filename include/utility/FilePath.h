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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <string>
#include <vector>


//using namespace boost::filesystem;

#pragma once
// this class is used to represent a file path
// it holds a private stl string or perhaps an array of string and 3 special strings
// both types of slashes can be given to it but they will be changed to a consistent form
// you can also specify file paths with the wild card character which will be used with in comparison operations
// .. will also be given a special meaning
/*
	FilePath(�fish_for_all.txt�) == FilePath( �fish*�) is true 
	FilePath(�fish_for_all.txt�) == FilePath( �fish*.txt�) is true
	FilePath(�fish*.txt�) == FilePath( �fish_for_all.txt�) is true
	FilePath(�fish_for_all.txt�) == FilePath( �*_for_all.txt�) is true
	
	You can only have wild cards in one of the FilePath strings
	A file path may have more than one wild card though if this is too hard to implement a single wild card may be excepted Brock let me know if you would like some help with the algorithm
*/

class FilePath
{
public:
   FilePath(void);

   ~FilePath(void);

   FilePath(const std::string &name);

   bool ok() //for testing/debugging
   {
      return !error;
   }

   //operator  FilePath ==  FilePath
   inline bool operator==(const FilePath &rhs) const {
      if (full_path == rhs.full_path) {
         return true;
      }
      //look for wild cards on the left side operand
      if (num_wild > 0) {
         //if the first char is a * then everything after it should be checked for equality
         //mainly used for file types like all txt files: *.txt
         size_t found = full_path.find("*");
         if (found == 0) {
            std::string end = full_path.substr(1);
            if (rhs.full_path.substr((rhs.full_path.length() - end.length())) == end) {
               return true;
            }
         }
            //if the first char is not a * then find every occurence of the text before the *
            //mainly used for folders like release* will find any occurence of the word release
         else {
            if (rhs.full_path.find(full_path.substr(0, found - 1)) != std::string::npos) {
               return true;
            }
         }
      }
      //look for wild cards on the right side operand
      if (rhs.num_wild > 0) {
         size_t found = rhs.full_path.find("*");
         if (found == 0) {
            std::string end = rhs.full_path.substr(1);
            if (full_path.substr((full_path.length() - end.length())) == end) {
               return true;
            }
         } else {
            if (rhs.full_path.find(full_path.substr(0, found - 1)) != std::string::npos) {
               return true;
            }
         }
      }
      return false;
   }

   std::string get_Full_Path() const; //returns the full path as a string
   std::string get_Type() const;   // returns the part after the . if there is not extension then return an empty string
   // for a file name like this foo.bar.txt this returns txt
   std::string get_Disk() const;   // this returns the disk if it is specified
   // FilePath(�C:king/crab.h�).get_Disk() -> C
   std::string remove_Last_Dir() const; // returns the popped value
   int get_Num_Dir() const;

   std::vector<std::string> list_Dirs() const;

   std::string get_Name() const {
      return file_name;
   }

   FilePath pop() const; //returns path with one less
   FilePath push(const FilePath &more) const; // adds a path to the end of this and returns the result


   FilePath cd(const FilePath &fp) const; // this assume you are in this and gives you the path that you
   // would end up in if you typed cd from that directory
   // FilePath(�C:king/crab/cake�).cd(�../pie�) ;
   // will give you �C:king/crab/pie�
   // if there is no directory pie in crab you will not get a
   // message as there is no way to tell with this class it is just a string manipulator
   // unlike the command line if you type
   /*
   FilePath(��).cd(�e:�) you will end up with �E:�
   FilePath(�C:king/crab/cake�).cd(�e�) ; will get you (�C:king/crab/cake/e�);
   FilePath(�C:king/crab/cake�).cd(�e:�) ; will get you (�E:�);
   */
   // also unlike cd you can cd into a file



private:

   std::string full_path;
   std::string drive;
   std::string type;
   std::vector<std::string> list_of_directories;
   int num_dir;
   std::string last_dir;
   int num_wild;
   bool error;
   std::string file_name; //for C:/foo/bar.txt, the file name is bar.txt


   //very private, only to be used in the constructor///////////
   int extract_disk(const std::string &name);                      //
   int extract_type(const std::string &name);                      //
   void extract_path(const std::string &name,
                     int &index,
                     int end);//
   void extract_pop(const std::string &name);                      //
   int get_Number_Of_Wild_Cards(const std::string &name);          //
   /////////////////////////////////////////////////////////////

};
