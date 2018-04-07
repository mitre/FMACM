#!python
# ================================================================================================ #
'''
Adapted from `Atlassian docs-plugin <https://bitbucket.org/ricebean/docs-plugin/wiki/REST_Interface_V2>`_.
'''

import argparse, base64, contextlib, getpass, httplib, ssl, urllib

def run( server , credFile , certFile , command , docFile , apiArgs ):
    # get user credentials
    if credFile is None:
        username = raw_input('Enter your user name: ')
        password = getpass.getpass('Enter the password for "%s": ' % username)
    else:
        with open( credFile , 'r' ) as f:
            username , password = [ line.strip() for line in f.readlines() ]
    
    # server authentication
    auth = base64.encodestring('%s:%s' % (username, password)).replace('\n', '')
    headers = {"X-Atlassian-Token": "nocheck", "Authorization": "Basic %s" % auth, "Content-Type": "application/json"}
    
    # select functionality based on input arguments
    meth = command.upper()
    subUrl = '/'.join(  urllib.quote(arg, '') for arg in apiArgs )
    body = None
    if command == 'put':
        assert 1 <= len(apiArgs) <= 2, 'This method requires 1 or 2 arguments'
        
        # Create Category: Create a new category (menu item) in the repository.
        # Method:    PUT
        # URL:       /rest/docs/2.0/repository/{categroy-name}
        
        #Create Doc: Upload and create a new doc archive (.zip / .jar) under the defined category.
        # Method:    PUT
        # URL:       /rest/docs/2.0/repository/{categroy-id}/{doc-name}
        if len(apiArgs) == 2:
            assert docFile is not None, 'This method requires a file path input (-f option)'
            body = open(docFile, "rb")
            
    elif command == 'post':
        assert 1 <= len(apiArgs) <= 2, 'This method requires 1 or 2 arguments'
        
        # Update Doc: Upload a doc archive (.zip / .jar) and UPDATE the specified (existing) doc.
        # Method:    POST 
        # URL:       /rest/docs/2.0/repository/{doc-key}
        if len(apiArgs) == 1:
            assert docFile is not None, 'This method requires a file path input (-f option)'
            body = open(docFile, "rb")
            print subUrl
        
        # Rename Doc: Rename a doc.
        # Method:    POST 
        # URL:       /rest/docs/2.0/repository/{doc-key}/{doc-name}
        else:
            response = raw_input("Are you sure you want to rename that documentation? (yes/[no]):")
            if not response.lower().strip().startswith('yes'):
                return
        
    elif command == 'delete':
        assert len(apiArgs) == 1, 'This method requires exactly 1 argument'
        
        # Delete Category: Delete a category (menu item).
        # Method:    DELETE
        # URL:       /rest/docs/2.0/repository/{categroy-id}
            
        # Delete Doc: Delete a specific doc.
        # Method:    DELETE 
        # URL:       /rest/docs/2.0/repository/{doc-key}
        response = raw_input("Are you sure you want to DELETE that documentation / category? (yes/[no]):")
        if not response.lower().strip().startswith('yes'):
            return
        response = raw_input("If this is an ENTIRE category, have you made sure you won't clobber someone else's documentation? (yes/[no]):")
        if not response.lower().strip().startswith('yes'):
            return
            
    else: # i.e. 'get'
        assert len(apiArgs) <= 1, 'This method requires at most 1 argument'
        
        # Get Repository Details: Return a json object representing the menu structure of the docs repository.
        # Method:    GET
        # URL:       /rest/docs/2.0/repository/
        
        # Load Doc Details: Get Details about a specific doc.
        # Method:    GET
        # URL:       /rest/docs/2.0/repository/{doc-key}
    
    # use the API via an SSL connection (passing in the MITRE certificate file)
    sslContext = ssl.create_default_context( cafile=certFile ) # need to handle SSL certificates
    with contextlib.closing( httplib.HTTPSConnection( server , context=sslContext ) ) as conn:
        conn.request(meth, "/rest/docs/2.0/repository/" + subUrl, body, headers)
        response = conn.getresponse()
        body = response.read()
    
    # finished...
    print( '''
RESPONSE
========

status = %s
reason = %s

body
----
%s
----
    ''' % ( response.status , response.reason , body ) )

# ================================================================================================ #

# The "__main__" thing
def main():
    parser = argparse.ArgumentParser( description='''
Automated Atlassian docs-plugin interaction via REST API v2.0

NOTE: this version of the API sometimes requires category and documentation names,
while other times it requires IDs/keys that it generated for you! To determine the
proper input arguments for each method, please see the API documentation:

https://bitbucket.org/ricebean/docs-plugin/wiki/REST_Interface_V2
''' )
    parser.add_argument( 'command' , choices=['get','put','post','delete'] , help='select API method' )
    parser.add_argument( '-s' , '--server' , default='huddle.mitre.org' , help='server URL [%(default)s]' )
    parser.add_argument( '-p' , '--credFile' , default=None ,
        help='two-line file with username and password (make sure to "chmod 600" this file!!) [%(default)s]' )
    parser.add_argument( '-t' , '--certFile' , default=None , help='path to MITRE-chain.pem file [%(default)s]' )
    parser.add_argument( '-f' , '--docFile' , default=None , help='sub-doc .jar/.zip file [%(default)s]' )
    parser.add_argument( 'apiArgs' , nargs='*' , default=None ,
        help='arguments to pass to the selected API method [%(default)s]' )
    run( **vars(parser.parse_args()) )

# ------------------------------------------------------------------------------------------------ #

# If script is called (NOT imported), run the main function
if __name__ == "__main__":
    main()

# ================================================================================================ #
