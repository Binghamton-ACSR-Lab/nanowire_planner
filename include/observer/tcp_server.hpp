//
// Created by acsr on 5/8/21.
//

#ifndef NANOWIREPLANNER_TCP_SERVER_HPP
#define NANOWIREPLANNER_TCP_SERVER_HPP

#include <atomic>
#include <brynet/base/AppStatus.hpp>
#include <brynet/base/Packet.hpp>
#include <brynet/net/EventLoop.hpp>
#include <brynet/net/ListenThread.hpp>
#include <brynet/net/SocketLibFunction.hpp>
#include <brynet/net/TcpConnection.hpp>
#include <brynet/net/TcpService.hpp>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <vector>

#include "observer.hpp"

namespace acsr {
    using namespace brynet;
    using namespace brynet::net;
    using namespace brynet::base;

    class TcpServer : public MessageDisplayer,public SolutionUpdateObserver{
    private:
        std::atomic_llong TotalSendLen = ATOMIC_VAR_INIT(0);
        std::atomic_llong TotalRecvLen = ATOMIC_VAR_INIT(0);

        std::atomic_llong SendPacketNum = ATOMIC_VAR_INIT(0);
        std::atomic_llong RecvPacketNum = ATOMIC_VAR_INIT(0);

        std::vector<TcpConnection::Ptr> clients;
        TcpService::Ptr service;

        std::atomic_bool run_flag{false};
        int _port;

    public:
        TcpServer(int port) : _port(port) {

        };

        virtual ~TcpServer() {
            //service->stopWorkerThread();
            run_flag = false;
        };

        void addClientID(const TcpConnection::Ptr &session) {
            clients.push_back(session);
        }

        void removeClientID(const TcpConnection::Ptr &session) {
            for (auto it = clients.begin(); it != clients.end(); ++it) {
                if (*it == session) {
                    clients.erase(it);
                    break;
                }
            }
        }

        size_t getClientNum() {
            return clients.size();
        }

        void broadCastPacket(const brynet::net::SendableMsg::Ptr &packet) {
            auto packetLen = packet->size();
            RecvPacketNum++;
            TotalRecvLen += packetLen;

            for (const auto &session : clients) {
                session->send(packet);
            }

            SendPacketNum += clients.size();
            TotalSendLen += (clients.size() * packetLen);
        }

        template<class F>
        void run(F f) {
            if (run_flag) {
                std::cout << "server is running\n";
                return;
            }
            run_flag = true;

            std::thread t([this, f] {
                brynet::net::base::InitSocket();

                service = TcpService::Create();
                auto mainLoop = std::make_shared<EventLoop>();
                auto listenThread = ListenThread::Create(false, "0.0.0.0", _port,
                                                         [mainLoop, this, &f](TcpSocket::Ptr socket) {
                                                             socket->setNodelay();
                                                             socket->setSendSize(32 * 1024);
                                                             socket->setRecvSize(32 * 1024);

                                                             auto enterCallback = [mainLoop, this, &f](
                                                                     const TcpConnection::Ptr &session) {
                                                                 mainLoop->runAsyncFunctor([session, this]() {
                                                                     addClientID(session);
                                                                     std::cout << session->getIP() << " connected\n";
                                                                 });

                                                                 session->setDisConnectCallback([mainLoop, this](
                                                                         const TcpConnection::Ptr &session) {
                                                                     std::cout << session->getIP() << " disconnected\n";
                                                                     mainLoop->runAsyncFunctor([session, this]() {
                                                                         removeClientID(session);
                                                                     });
                                                                 });


                                                                 /*
                                                                 session->setHighWaterCallback([]() {
                                                                                                   std::cout << "high water" << std::endl;
                                                                                               },
                                                                                               1024 * 1024 * 100);*/

                                                                 session->setDataCallback([mainLoop, &f, this](
                                                                         brynet::base::BasePacketReader &reader) {
                                                                     auto buffer = reader.currentBuffer();
                                                                     f(buffer, reader.size());
                                                                     reader.consumeAll();
                                                                 });
                                                             };
                                                             ConnectionOption option;
                                                             option.enterCallback.emplace_back(enterCallback);
                                                             option.maxRecvBufferSize = 1024 * 1024;
                                                             service->addTcpConnection(std::move(socket), option);
                                                         });

                listenThread->startListen();
                service->startWorkerThread(2);
                while (run_flag) {
                    mainLoop->loop(1);
                }
                service->stopWorkerThread();
            });
            t.detach();
        }

        void stopServer() {
            run_flag = false;
        }

        void displayMessage(const std::string &msg) override {
            auto new_msg(msg);
            if (new_msg.substr(new_msg.length() - 2) != "\r\n") {
                new_msg += "\r\n";
            }
            for (auto &client:clients) {
                client->send(new_msg.c_str(), new_msg.length());
            }

        }

        void onSolutionUpdate(const std::vector<Eigen::VectorXd> &forward_states,
                              const std::vector<Eigen::VectorXd> &reverse_states,
                              const std::vector<Eigen::VectorXd> &connect_states,
                              const std::vector<Eigen::VectorXd> &forward_control,
                              const std::vector<Eigen::VectorXd> &reverse_control,
                              const std::vector<Eigen::VectorXd> &connect_control,
                              const std::vector<double> &forward_durations,
                              const std::vector<double> &reverse_durations,
                              const std::vector<double> &connect_durations,
                              const std::string& solution_string) override {
            auto d = std::accumulate(forward_durations.begin(),forward_durations.end(),0.0);
            d = std::accumulate(reverse_durations.begin(),reverse_durations.end(),d);
            d = std::accumulate(connect_durations.begin(),connect_durations.end(),d);
            std::string  s= "solution updated, cost: "+std::to_string(d) +"\r\n";
            for (auto &client:clients) {
                client->send(s.c_str(), s.length());
            }

        }


    };
}




#endif //NANOWIREPLANNER_TCP_SERVER_HPP
