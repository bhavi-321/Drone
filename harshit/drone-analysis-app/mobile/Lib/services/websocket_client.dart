import 'package:websocket_listener/websocket_listener.dart';

class WSClient {
  WebSocketListener? _ws;
  void connect(String userId) {
    _ws = WebSocketListener('ws://YOUR_BACKEND:8000/ws/$userId');
    _ws!.onData = (data) { print('WS: $data'); };
    _ws!.onError = (err) { print('WS err: $err'); };
  }
  void close() => _ws?.close();
}