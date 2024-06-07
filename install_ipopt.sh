# install cppad from apt
apt install cppad

# download ipopt
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.14.4.tar.gz
tar -xf Ipopt-3.14.4.tar.gz

# build and install mumps
cd Ipopt-releases-3.14.4/
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps
./get.Mumps
mkdir build
cd build
../configure
make
make install

# build and install ipopt
cd ../..
mkdir build
cd build
../configure
make
make install

# fix naming issue from ipopt installation
cd /usr/local/include
cp -r coin-or coin
