#include <iostream>
#include <boost/python.hpp>
#include <QtNetwork>
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"

using namespace boost::python;

#define MAX_ROBOTS 12

template<typename T>
inline
std::vector< T > py_list_to_std_vector( const boost::python::object& iterable )
{
    return std::vector< T >( boost::python::stl_input_iterator< T >( iterable ),
                             boost::python::stl_input_iterator< T >( ) );
}

template <class T>
inline
boost::python::list std_vector_to_py_list(std::vector<T> vector) {
    typename std::vector<T>::iterator iter;
    boost::python::list list;
    for (iter = vector.begin(); iter != vector.end(); ++iter) {
        list.append(*iter);
    }
    return list;
}

class SSL_Sender{
public:
    SSL_Sender();
    SSL_Sender(const SSL_Sender&);
    void send(int isYellow, int wheelsSpeed,
        std::vector<double> w1, std::vector<double> w2,
        std::vector<double> w3, std::vector<double> w4,
        std::vector<double> vx, std::vector<double> vy, std::vector<double> vw,
        std::vector<double> kx, std::vector<double> kz, std::vector<double> sp);
    void sendWheels(int isYellow,
        boost::python::object& w1, boost::python::object& w2,
        boost::python::object& w3, boost::python::object& w4,
        boost::python::object& kx, boost::python::object& kz, boost::python::object& sp);
    void sendVels(int isYellow,
        boost::python::object& vx, boost::python::object& vy, boost::python::object& vw,
        boost::python::object& kx, boost::python::object& kz, boost::python::object& sp);

private:
    QUdpSocket udpsocket;
    QHostAddress _addr = QHostAddress("127.0.0.1");
    quint16 _port = 20011;
};

SSL_Sender::SSL_Sender(const SSL_Sender&){
    _addr = QHostAddress("127.0.0.1");
    _port = 20011;
}

SSL_Sender::SSL_Sender(){
    _addr = QHostAddress("127.0.0.1");
    _port = 20011;
}

void SSL_Sender::sendVels(int isYellow,
    boost::python::object& vx, boost::python::object& vy, boost::python::object& vw,
    boost::python::object& kx, boost::python::object& kz, boost::python::object& sp){
    int size = boost::python::len(vx);
    send(isYellow, 0,
        std::vector<double>(size,0), std::vector<double>(size,0),
        std::vector<double>(size,0), std::vector<double>(size,0),
        py_list_to_std_vector<double>(vx), py_list_to_std_vector<double>(vy),
        py_list_to_std_vector<double>(vw), py_list_to_std_vector<double>(kx),
        py_list_to_std_vector<double>(kz), py_list_to_std_vector<double>(sp));
}

void SSL_Sender::sendWheels(int isYellow,
    boost::python::object& w1, boost::python::object& w2,
    boost::python::object& w3, boost::python::object& w4,
    boost::python::object& kx, boost::python::object& kz, boost::python::object& sp){
    int size = boost::python::len(w1);
    send(isYellow, 1,
        py_list_to_std_vector<double>(w1), py_list_to_std_vector<double>(w2),
        py_list_to_std_vector<double>(w3), py_list_to_std_vector<double>(w4),
        std::vector<double>(size,0), std::vector<double>(size,0), std::vector<double>(size,0),
        py_list_to_std_vector<double>(kx), py_list_to_std_vector<double>(kz),
        py_list_to_std_vector<double>(sp));
}

void SSL_Sender::send(int isYellow, int wheelsSpeed,
    std::vector<double> w1, std::vector<double> w2,
    std::vector<double> w3, std::vector<double> w4,
    std::vector<double> vx, std::vector<double> vy, std::vector<double> vw,
    std::vector<double> kx, std::vector<double> kz, std::vector<double> sp){
    if(!( w1.size()==w2.size() && w2.size()==w3.size() && w3.size()==w4.size() &&
          w4.size()==kx.size() && kx.size()==kz.size() && kz.size()==sp.size() &&
          sp.size()==vx.size() && vx.size()==vy.size() && vy.size()==vw.size() )){
        std::cerr << "Error: input vectors have to have the same size." << std::endl;
        return;
    }
    QByteArray dgram;
    grSim_Packet packet;
    bool yellow = isYellow != 0 ? true : false;
    packet.mutable_commands()->set_isteamyellow(yellow);
    packet.mutable_commands()->set_timestamp(0.0);
    for(int i=0; i<w1.size(); i++){
        grSim_Robot_Command* command1 = packet.mutable_commands()->add_robot_commands();
        command1->set_id(i);
        command1->set_wheelsspeed(wheelsSpeed);
        command1->set_wheel1(w1[i]);
        command1->set_wheel2(w2[i]);
        command1->set_wheel3(w3[i]);
        command1->set_wheel4(w4[i]);
        command1->set_veltangent(vy[i]);
        command1->set_velnormal(vx[i]);
        command1->set_velangular(vw[i]);
        command1->set_kickspeedx(kx[i]);
        command1->set_kickspeedz(kz[i]);
        command1->set_spinner(sp[i]);
    }
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    udpsocket.writeDatagram(dgram, _addr, _port);
}

BOOST_PYTHON_MODULE(sender)
{
    class_< SSL_Sender >("SSL_Sender")
      .def("sendWheels", &SSL_Sender::sendWheels, args("isYellow","w1","w2","w3","w4","kx","kz","sp"))
      .def("sendVels", &SSL_Sender::sendVels, args("isYellow","vx","vy","vz","kx","kz","sp"))
      ;
}

// int main(int argc, char** argv){
//     SSL_Sender ssl_sender;
//     std::vector<double> w1, w2, w3, w4, kx, kz, sp;
//     for(int i=0; i<3; i++){
//         w1.push_back(10);
//         w2.push_back(10);
//         w3.push_back(10);
//         w4.push_back(10);
//         kx.push_back(0);
//         kz.push_back(0);
//         sp.push_back(0);
//     }
//     ssl_sender.send(1, w1, w2, w3, w4, kx, kz, sp);
//     return 0;
// }
