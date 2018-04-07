import sys
import base64
import httplib
import urllib
import zipfile
import os
import atlassianDocsREST

#
# Examine this more closely
#  http://httplib2.readthedocs.org/en/stable/libhttplib2.html#examples
#
# Also, why can't this use ssh keys for auth?
#

def recursive_zip(zipf, directory, folder = ""):
   for item in os.listdir(directory):
      if os.path.isfile(os.path.join(directory, item)):
         zipf.write(os.path.join(directory, item), folder + os.sep + item)
      elif os.path.isdir(os.path.join(directory, item)):
         recursive_zip(zipf, os.path.join(directory, item), folder + os.sep + item)
 
if __name__ == '__main__':
    """
    Zip the contents of ../docs/html/* into a file aaesimdocs.zip. Post the zip file to huddle.mitre.org
    
    Input Parameters:
    """    
    # get params
    docFile = "aaesimdocs.zip"
    uploadKey = "c1006-d1008"
    categoryName = "AAESim"
    itemName = 'branch:master'

    # zip the documentation
    zipf = zipfile.ZipFile(docFile, "w", compression=zipfile.ZIP_DEFLATED )
    path = '../docs/html'
    recursive_zip(zipf, path) #leave the first folder as None, as path is root.
    zipf.close()

    # call Tudor's script
    atlassianDocsREST.run( 'huddle.mitre.org' , '/home/sbowman/login_credentials.private' , '/home/sbowman/MITRE-chain.crt' , 'post' , docFile, [uploadKey] )
