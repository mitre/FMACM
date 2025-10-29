//  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESSED OR IMPLIED WARRANTIES,
//  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//  FITNESS  FOR A PARTICULAR  PURPOSE ARE  DISCLAIMED.  IN NO  EVENT SHALL  THE
//  APACHE SOFTWARE  FOUNDATION  OR ITS CONTRIBUTORS  BE LIABLE FOR  ANY DIRECT,
//  INDIRECT, INCIDENTAL, SPECIAL,  EXEMPLARY, OR CONSEQUENTIAL  DAMAGES (INCLU-
//  DING, BUT NOT LIMITED TO, PROCUREMENT  OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  OF USE, DATA, OR  PROFITS; OR BUSINESS  INTERRUPTION)  HOWEVER CAUSED AND ON
//  ANY  THEORY OF LIABILITY,  WHETHER  IN CONTRACT,  STRICT LIABILITY,  OR TORT
//  (INCLUDING  NEGLIGENCE OR  OTHERWISE) ARISING IN  ANY WAY OUT OF THE  USE OF
//  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * This is just an aggregator header file.
 */

#if !defined(aaesim_CPPMANIFEST_H)
#define aaesim_CPPMANIFEST_H

#if defined(aaesim_CPPMANIFEST_HAVE_PRAGMA_ONCE)
#pragma once
#endif

#include "build_info.h"

namespace aaesim {
namespace cppmanifest {
static std::string BUILDINFO_CLI_FLAG = "--buildinfo";
static std::string CLEAN_STR = "CLEAN";
static std::string DIRTY_STR = "DIRTY";

static std::string GetUserName() { return aaesim_CPPMANIFEST_USERNAME_STR; }

static std::string GetBuildTimeStamp() { return aaesim_CPPMANIFEST_BUILDTIMESTAMP_STR; }

static std::string GetBuildCompilerVersion() { return aaesim_CPPMANIFEST_BUILD_CXX_VERSION_STR; }

static std::string GetBuildSystemVersion() { return aaesim_CPPMANIFEST_BUILDSYSTEMVERSION_STR; }

static std::string GetBuildSystemName() { return aaesim_CPPMANIFEST_BUILDSYSTEMNAME_STR; }

static std::string GetBuildSystemProcessor() { return aaesim_CPPMANIFEST_BUILDSYSTEMPROCESSOR_STR; }

static std::string GetBuildHostName() { return aaesim_CPPMANIFEST_BUILDHOSTNAME_STR; }

static std::string GetGitBranch() { return aaesim_CPPMANIFEST_GIT_BRANCH; }

static std::string GetGitHash() { return aaesim_CPPMANIFEST_GIT_HASH; }

static std::string GetGitTag() { return aaesim_CPPMANIFEST_GIT_TAG; }

static bool GetGitIsClean() {
   if (CLEAN_STR.compare(aaesim_CPPMANIFEST_GIT_LOCAL_CHANGES) == 0)
      return 1;
   else
      return 0;
}

static std::string GetVersion() {
   std::string version_str(aaesim_CPPMANIFEST_VERSION_STR);
   bool stripLastChar = version_str.find("-") == version_str.length() - 1;
   if (stripLastChar) {
      // dump the last char
      version_str.resize(version_str.find("-"));
   }
   return version_str;
}

static void PrintMetaData() {
   std::cout << "Build version: " << GetVersion() << std::endl;
   std::cout << "Created-by: " << GetUserName() << std::endl;
   std::cout << "Created-on: " << GetBuildTimeStamp() << std::endl;
   std::cout << "Built with GCC version: " << GetBuildCompilerVersion() << std::endl;
   std::cout << "Built on system name: " << GetBuildSystemName() << std::endl;
   std::cout << "Built on system processor: " << GetBuildSystemProcessor() << std::endl;
   std::cout << "Built with system ver: " << GetBuildSystemVersion() << std::endl;
   std::cout << "Built on system hostname: " << GetBuildHostName() << std::endl;
   std::cout << "Built from git branch: " << GetGitBranch() << std::endl;
   if (GetGitIsClean())
      std::cout << "Built from git hash: " << GetGitHash() << std::endl;
   else
      std::cout << "Built from git hash: " << GetGitHash() << "-" << DIRTY_STR << std::endl;
}

static void ProcessCommandLineArgs(int argc, char *argv[]) {
   for (auto i = 0; i < argc; ++i) {
      if (BUILDINFO_CLI_FLAG.compare(argv[i]) == 0) {
         PrintMetaData();
         break;
      }
   }
}
}  // namespace cppmanifest
}  // namespace aaesim

#endif
/*
 * Do not put content below here.
 */
