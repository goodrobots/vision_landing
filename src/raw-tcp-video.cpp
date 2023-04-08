#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <signal.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

int server_socket, client_socket;
struct sockaddr_in server_address, client_address;

double lastTime;

void check_exit();
double CLOCK();

void accept_connection() {
    cout << "Waiting for video streamer...\n";

    socklen_t client_address_size = sizeof(client_address);

    for(;;) {
        client_socket = accept(server_socket, (struct sockaddr *)&client_address, &client_address_size);
        if (client_socket < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                cout << "Retrying accept\n";
                check_exit();
                continue;
            }
            std::cerr << "Accept failed\n";
            exit(1);

        } else {
            break; // Connected
        }
    }

    cout << "Video streamer connected.\n";
}

void raw_tcp_reconnect() {
    cout << "Client disconnected. Reconnecting...\n";
    close(client_socket);
    accept_connection();
}

void init_video_socket() {
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        std::cerr << "Could not create socket\n";
        exit(1);
    }

    // Prepare the sockaddr_in structure
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(6000);

    // Bind the socket to the address and port
    if (bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        std::cerr << "Bind failed\n";
        exit(1);
    }

    // Listen for incoming connections
    listen(server_socket, 3);
 
    accept_connection();
}

void close_video_socket() {
    // Close sockets
    close(client_socket);
    close(server_socket);
}

// TODO: OPT: Test working directly with yuvMat
Mat raw_tcp_get_image(uint64_t &ts, float &yaw) {
    std::streamsize bytes;
    int consecutiveErrors = 0;
    for(;;) {
        int w, h, size;

        bool err = false;

        if(!recv(client_socket, &w, sizeof(w), 0)) {
            raw_tcp_reconnect();
            continue;
        }

        if(
            recv(client_socket, &h, sizeof(h), 0) <= 0
            || recv(client_socket, &ts, sizeof(ts), 0) <= 0
            || recv(client_socket, &yaw, sizeof(yaw), 0) <= 0
            || recv(client_socket, &size, sizeof(size), 0) <= 0
        ) {
            err = true;
            size = 0;
        }

        char buffer[size];

        int bytes_received = 0;
        if(!err) {
            // TODO: if socket is blocking, while is not necessary
            while (bytes_received < size) {
                int bytes = recv(client_socket, buffer + bytes_received, size - bytes_received, 0);
                if (bytes == -1) {
                    err = true;
                    break;
                    /*
                    cout << ".";
                    continue;
                    */

                } else if (!bytes) {
                    accept_connection();
                    continue;
                }
                bytes_received += bytes;
            }
        }

        if(err) {
            check_exit();
            if(!consecutiveErrors) cout << "Retrying...\n";
            consecutiveErrors++;

            if(consecutiveErrors > 100) sleep(1); // Client probably paused => relax CPU

        } else {
            consecutiveErrors = 0;

            double now = CLOCK();

            cout << "Got image";

            if(lastTime) cout << ", ts: " << ts << ", ms: " << (now - lastTime);
            lastTime = now;

            cout << ", bytes: " << bytes_received << "\n";
            cout.flush();

            Mat yuvMat(h + h / 2, w, CV_8UC1, buffer);
            Mat rgbMat;
            cvtColor(yuvMat, rgbMat, COLOR_YUV2BGR_I420);

            return rgbMat;
        }
    }
}
