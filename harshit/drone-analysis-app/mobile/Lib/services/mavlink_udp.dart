import 'dart:typed_data';
import 'package:udp/udp.dart';

class MavlinkUDP {
  UDP? _socket;

  Future<void> start({int port = 14550}) async {
    _socket = await UDP.bind(Endpoint.any(port: Port(port)));
    _socket!.asStream().listen((datagram) {
      if (datagram == null) return;
      final data = datagram.data;
// parse MAVLink binary or use mavlink parsing library
// for a start, treat as JSON messages if bridge sends JSON
      final text = String.fromCharCodes(data);
      print('UDP: $text');
    });
  }

  Future<void> stop() async => await _socket?.close();
}