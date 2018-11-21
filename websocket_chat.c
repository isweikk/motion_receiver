/*
 * Copyright (c) 2014 Cesanta Software Limited
 * All rights reserved
 */

#include "mongoose.h"
#include "motion_motor_ctrl.h"

static sig_atomic_t s_signal_received = 0;
static const char *s_http_port = "8000";
static struct mg_serve_http_opts s_http_server_opts;

struct mg_connection *nc;


static void signal_handler(int sig_num) {
  signal(sig_num, signal_handler);  // Reinstantiate signal handler
  s_signal_received = sig_num;
}

static int is_websocket(const struct mg_connection *nc) {
  return nc->flags & MG_F_IS_WEBSOCKET;
}

static void broadcast(struct mg_connection *nc, const struct mg_str msg) {
  struct mg_connection *c;
  char buf[500];
  char addr[32];
  mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
                      MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);

  snprintf(buf, sizeof(buf), "%s %.*s", addr, (int) msg.len, msg.p);
  printf("%s\n", buf); /* Local echo. */
  for (c = mg_next(nc->mgr, NULL); c != NULL; c = mg_next(nc->mgr, c)) {
    if (c == nc) continue; /* Don't send to the sender. */
    mg_send_websocket_frame(c, WEBSOCKET_OP_TEXT, buf, strlen(buf));
  }
}

int handle_msg_from_client(struct mg_connection *nc, const struct mg_str msg) {
    int len = 0, i = 0;
    char recv_buf[500] = {0};

    printf("recv %d: %s\n", (int)msg.len, msg.p);
    len = mg_base64_decode((const unsigned char *)msg.p, msg.len, recv_buf);

    printf("decode:");
    for (i=0; i<len; i++){
        printf("%02x ", 0xff & recv_buf[i]);
    }
    printf("\n");
    printf("%d bytes msg ok", len);

    if(len >= 13){
        if(MCSendMotorCmdEx(recv_buf) == 0) {
            return 0;
        }
    }
    printf("%p\n", nc);

    return -1;
}

int send_msg_to_client(const char *cmd) {
    struct mg_connection *c;
    //int len = 0, i = 0;
    char send_buf[500] = {0};

    mg_base64_encode((const unsigned char *)cmd, 13, send_buf);
    printf("encode over %d:%s\n", (int)strlen(send_buf), send_buf);
    for (c = mg_next(nc->mgr, NULL); c != NULL; c = mg_next(nc->mgr, c)) {
        if (c == nc) continue; /* Don't send to the sender. */
            mg_send_websocket_frame(c, WEBSOCKET_OP_TEXT, send_buf, strlen(send_buf));
    }
    return 0;
}


static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
  switch (ev) {
    case MG_EV_WEBSOCKET_HANDSHAKE_DONE: {
      /* New websocket connection. Tell everybody. */
      broadcast(nc, mg_mk_str("++ joined"));
      break;
    }
    case MG_EV_WEBSOCKET_FRAME: {
      struct websocket_message *wm = (struct websocket_message *) ev_data;
      /* New websocket message. Tell everybody. */
      struct mg_str d = {(char *) wm->data, wm->size};
      handle_msg_from_client(nc, d);
      //broadcast(nc, d);
      break;
    }
    case MG_EV_HTTP_REQUEST: {
      mg_serve_http(nc, (struct http_message *) ev_data, s_http_server_opts);
      break;
    }
    case MG_EV_CLOSE: {
      /* Disconnect. Tell everybody. */
      if (is_websocket(nc)) {
        broadcast(nc, mg_mk_str("-- left"));
      }
      break;
    }
  }
}

int main(void) {
  struct mg_mgr mgr;
  //struct mg_connection *nc;

  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
  setvbuf(stdout, NULL, _IOLBF, 0);
  setvbuf(stderr, NULL, _IOLBF, 0);

  mg_mgr_init(&mgr, NULL);

  MotionCtrlStartup();
  nc = mg_bind(&mgr, s_http_port, ev_handler);
  if (nc == NULL){
    perror("bind faild");
    exit(0);
  }
  mg_set_protocol_http_websocket(nc);
  s_http_server_opts.document_root = ".";  // Serve current directory
  s_http_server_opts.enable_directory_listing = "no";

  printf("Started on port %s\n", s_http_port);
  while (s_signal_received == 0) {
    mg_mgr_poll(&mgr, 20);
  }
  mg_mgr_free(&mgr);

  return 0;
}
