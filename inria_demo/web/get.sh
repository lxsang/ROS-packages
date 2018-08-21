#! /bin/bash

# fetch the source
set +e
rm -r ant-http
set -e
git clone https://github.com/lxsang/ant-http
#set +e
#rm -r ./bin
#set -e
# sed the make file
sed -i 's/USE_DB=TRUE/USE_DB=FALSE/g' ant-http/Makefile
sed -i 's/USE_SSL = TRUE/USE_SSL = FALSE/g' ant-http/Makefile
sed -i 's/BUILDIRD=\/opt\/www/BUILDIRD=..\/bin/g' ant-http/Makefile
sed -i 's/PLUGINS=	pluginsman.$(EXT) wterm.$(EXT) nodedaemon.$(EXT) wsimg.$(EXT)/PLUGINS=/g' ant-http/Makefile
# make the bin
mkdir -p ./bin/plugins
cd ./ant-http && make && cd ../
rm -r ant-http
cp config.ini ./bin
mkdir ./bin/database ./bin/htdocs ./bin/tmp