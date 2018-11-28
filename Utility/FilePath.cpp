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

#include "utility/FilePath.h"
#include "utility/MoreStringFunctions.h"

using namespace std;


FilePath::FilePath(void) {
}

//------------------------------------------
FilePath::~FilePath(void) {
}

//------------------------------------------
FilePath::FilePath(const string &name) {
   error = false;
   int end;
   int index;

   //make sure there is a file path
   if (name.length() == 0) {
      error = true;
   }

   full_path = name;

   extract_pop(name); //sets the popped value
   end = extract_type(name); //returns the index of the type
   index = extract_disk(name); //returns the index of the disk

   extract_path(name, index, end);

   num_wild = get_Number_Of_Wild_Cards(name);

   if (list_of_directories.size() == 0) {
      file_name = "";
   } else {
      file_name = list_of_directories.at(list_of_directories.size() - 1);
   }

   //return error if there is a space at the end of the path
   size_t find_space;
   find_space = name.find_last_of(" ");
   if (find_space != string::npos) {
      if (find_space == name.length() - 1) {
         error = true;
      }
   }

}

//------------------------------------------------------------------------------------------
//returns the full path as a string
string FilePath::get_Full_Path() const {
   return full_path;
}

//------------------------------------------------------------------------------------------
//returns the file type for the path
//for example C:/temp/test.cpp returns cpp and C:/temp returns ""
string FilePath::get_Type() const {
   return type;
}

//------------------------------------------------------------------------------------------
//returns the disk for the file path
//for example C:/temp would return C
string FilePath::get_Disk() const {
   return drive;
}

//------------------------------------------------------------------------------------------
//returns the private variable last dir which represents the last directory or file in the path
string FilePath::remove_Last_Dir() const {
   return last_dir;
}

//------------------------------------------------------------------------------------------
//returns the number of directories
int FilePath::get_Num_Dir() const {
   return num_dir;
}

//------------------------------------------------------------------------------------------
//returns a vector listing all directories in the file path
vector<string> FilePath::list_Dirs() const {
   return list_of_directories;
}

//------------------------------------------------------------------------------------------
//returns the file path when it is popped up one level
//for example C:/temp/test popped would be C:/temp
FilePath FilePath::pop() const {
   FilePath out = FilePath(last_dir);
   return out;
}

//------------------------------------------------------------------------------------------
//returns a file path that represents the initial path plus the pushed path specified
//for example fp.push("/temp") would be C:/temp if fp is C: 
FilePath FilePath::push(const FilePath &more) const {
   string pushed;
   string last = full_path.substr(full_path.length() - 1, 1);
   if (last == "\\" || last == "/") {
      pushed = full_path + more.full_path;
   } else {
#ifdef _MSC_VER
      pushed = full_path + "\\" + more.full_path;
#else
      pushed = full_path + "/" + more.full_path;
#endif
   }
   FilePath out = FilePath(pushed);
   return out;
}

//------------------------------------------------------------------------------------------
// returns a FilePath object that is in relation to the given path
//for example, one could do cd .. or cd ../ or cd ../temp to pop up
//one could also do cd temp/moretemp to push into a further down directory
FilePath FilePath::cd(const FilePath &fp) const {
   string full = fp.full_path;
   string full1 = full_path;
   FilePath temp = FilePath(full1);
   FilePath out, temp2;
   if (full.substr(0, 2) == "..") {
      if (full == ".." || full == "../" || full == "..\\") {
         out = temp.pop();
      } else if (full.substr(0, 3) == "..\\" || full.substr(0, 3) == "../") {
         temp2 = temp.pop();
         out = temp2.push(full.substr(3));
      } else {
         size_t col = full.find_first_of(":");
         int len = full.length();
         if (col != string::npos && col == len - 1) {
            out = fp;
         } else {
            out = temp;
         }
      }
   } else {
      out = temp.push(fp);
   }

   return out;
}

//------------------------------------------------------------------------------------------
//returns the index of the : representing a drive in the file path
//example- C:/temp returns 2
//if the path is C: then there are no directories and 10000 is returned
int FilePath::extract_disk(const string &name) {
   if (name.find_first_of(":") != string::npos) {
      drive = name.substr(0, name.find_first_of(":"));
      if ((name.length() - 1) == name.find_first_of(":")) {
         error = true; //return error if path is just C:
         return -1; // added by ADM jan 24 2011
      } else {
         return name.find_first_of(":") + 1;
      }
   } else {
      drive = "";
      return 0;
   }

}

//------------------------------------------------------------------------------------------
//returns the index of dot that represents the file type like .txt
//also sets the private variable type to the file type: like txt
//if no file type is found, ie its a folder, type is set to ""
int FilePath::extract_type(const string &name) {
   size_t found;
   found = name.find_last_of(".");
   if (found == string::npos) {
      type = "";
      return name.length();
   } else {
      //a path cannot have a period at the end
      if (found == name.length() - 1) {
         error = true;
      }
      type = name.substr(found + 1);
      return int(found);
   }
}

//------------------------------------------------------------------------------------------
//finds the number of directories in the path name and creates an array of all directories
//in the path
void FilePath::extract_path(const string &name,
                            int &index,
                            int end) {
#ifdef _MSC_VER
   string temp = "";
     string folder = "";
   string first = name.substr(index,1);
   num_dir = 0;
   if(first=="/" || first=="\\")
   {
      index++;
   }
     while(index < end)
   {
       //check for illegal characters in the file path
       if(name.substr(index,1)=="?" || name.substr(index,1)=="<" || name.substr(index,1)==">" || name.substr(index,1)=="|" || name.substr(index,1)=="\"")
      {
          error = true;
       }
       if(name.substr(index,1)=="/" || name.substr(index,1)=="\\")
      {
          list_of_directories.push_back(folder);
            num_dir++;
            folder = "";

            if(index==end-1) break;
       }
       else
      {
            folder = folder + name.substr(index,1);
            if(index==end-1)
         {
            list_of_directories.push_back(folder);
            num_dir++;
            folder = "";
            }
       }
       index++;
   }
#else
   num_dir = 0;
   string temp = name;
   while (index < end) {
      size_t slash = temp.find_first_of("/");
      string directory;
      if (slash != string::npos) {
         directory = name.substr(index, slash + 1);
         list_of_directories.push_back(directory);
         num_dir++;
         index += directory.size();
         temp = name.substr(index);
      } else {
         size_t dot = temp.find_first_of(".");
         directory = name.substr(index, dot);
         list_of_directories.push_back(directory);
         num_dir++;
         index += directory.size();
         temp = name.substr(index);
      }
   }
#endif
}

//------------------------------------------------------------------------------------------
//sets the private variable last_dir to whatever the path is when it is popped up one
void FilePath::extract_pop(const string &rname) {
#ifdef _MSC_VER
   string name = MoreStringFunctions::find_and_replace(rname, "/", "\\");
   size_t len = name.length();
   size_t col = name.find_first_of(":");
   size_t slash = name.find_last_of("\\");
   size_t new_slash;

   if(slash!=string::npos)
   {
      if(name.substr(0, 1)=="\\"&& len == 1 ) //takes care of root: /
      {
         last_dir = name;
      }
      if(slash==len-1 && len > 1) //found a slash at the end of the string
      {
         new_slash = name.find_last_of("\\", slash-1);
         if(new_slash!=string::npos)
         {
            last_dir = name.substr(0, new_slash);
         }
         else last_dir = name;
      }
      else
      {
         if(len > 1)
         {
            last_dir = name.substr(0, slash); //takes care of C:/test/text/more.txt
         }
      }
   }
   else
   {
      if(col == len-1)
      {
         last_dir = name; //takes care of C:, C:file.txt
      }
      else
        last_dir = name.substr(0, col+1);
   }
#else
   // Find the current directory path
   size_t slash = rname.find_last_of("/");
   last_dir = rname.substr(0, slash + 1);
#endif
}

//------------------------------------------------------------------------------------------
//returns the number of wild cards in the path
int FilePath::get_Number_Of_Wild_Cards(const string &name) {
   int len = full_path.length();
   int index = 0;
   int number = 0;
   while (index < len) {
      if (full_path.substr(index, 1) == "*") {
         number++;
      }
      index++;
   }
   return number;
}
