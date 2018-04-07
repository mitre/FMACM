# run the python tool that posts docs to huddle.mitre.org
cd /devel/aaesim/sbowman/aaesim-master
git pull
make doc
cd .python
. /home/sbowman/go/myApps/py279ve/bin/activate
python docs-upload.py
