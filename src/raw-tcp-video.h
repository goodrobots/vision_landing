// ---  Raw Video Stream ---

// This is an alternative image grabber that raw images send via tcp
// To enable, use input arg = "raw-tcp"

void accept_connection();
void raw_tcp_reconnect();
void init_video_socket();
void close_video_socket();
Mat raw_tcp_get_image(uint64_t &ts, float &yaw);
