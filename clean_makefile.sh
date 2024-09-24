#!bin/sh

BASE_DIR=${dirname ${readlink -f $0}}
echo "BASE_DIR="$BASE_DIR

# cd $BASE_DIR/build
# make clean
# cd $BASE_DIR

rm -rf $BASE_DIR/build

mkdir build

cd $BASE_DIR/build

cmake ..

make -j4




