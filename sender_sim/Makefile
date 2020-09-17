# location of the Python header files

PYTHON_VERSION = 3.6m
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)

# location of the Boost Python include files and library

BOOST_INC = /usr/include
BOOST_LIB = /usr/lib

# compile mesh classes
TARGET = sender

all: main.cpp
	g++ -I$(PYTHON_INCLUDE) -I$(BOOST_INC) -I/usr/include/qt4/QtNetwork -I/usr/include/qt4/ -fPIC -shared -Wl,--export-dynamic main.cpp grSim_Commands.pb.cc grSim_Packet.pb.cc grSim_Replacement.pb.cc -lQtCore -lQtNetwork -pthread -lprotobuf -lpthread -L$(BOOST_LIB) -lboost_python-py36 -L/usr/lib/python$(PYTHON_VERSION)/config -lpython$(PYTHON_VERSION) -o $(TARGET).so

test: main.cpp
	g++ -I$(PYTHON_INCLUDE) -I/usr/include/qt4/QtNetwork -I/usr/include/qt4/ -fPIC main.cpp grSim_Commands.pb.cc grSim_Packet.pb.cc grSim_Replacement.pb.cc -lQtCore -lQtNetwork -pthread -lprotobuf -lpthread -L$(BOOST_LIB) -lboost_python-py36 -L/usr/lib/python$(PYTHON_VERSION)/config -lpython$(PYTHON_VERSION) -o test
