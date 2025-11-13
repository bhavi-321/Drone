import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:http/http.dart' as http;

void main() {
  runApp(const DroneApp());
}

class DroneApp extends StatelessWidget {
  const DroneApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Drone Analysis App',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const DroneDashboard(),
    );
  }
}

class DroneDashboard extends StatefulWidget {
  const DroneDashboard({super.key});

  @override
  State<DroneDashboard> createState() => _DroneDashboardState();
}

class _DroneDashboardState extends State<DroneDashboard> {
  Map<String, dynamic>? telemetry;
  bool loading = true;

  Future<void> fetchTelemetry() async {
    final uri = Uri.parse("http://127.0.0.1:8000/telemetry/latest");
    try {
      final response = await http.get(uri);
      if (response.statusCode == 200) {
        setState(() {
          telemetry = json.decode(response.body);
          loading = false;
        });
      } else {
        setState(() => loading = false);
      }
    } catch (e) {
      print("Error: $e");
      setState(() => loading = false);
    }
  }

  @override
  void initState() {
    super.initState();
    fetchTelemetry();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Drone Telemetry Dashboard"),
        centerTitle: true,
      ),
      body: loading
          ? const Center(child: CircularProgressIndicator())
          : telemetry == null
              ? const Center(child: Text("No telemetry data found"))
              : Padding(
                  padding: const EdgeInsets.all(20.0),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text("📍 Latitude: ${telemetry!['latitude']}"),
                      Text("📍 Longitude: ${telemetry!['longitude']}"),
                      Text("📏 Altitude: ${telemetry!['altitude']} m"),
                      Text("⚡ Battery: ${telemetry!['battery']}%"),
                      Text("🚀 Velocity: ${telemetry!['velocity']} m/s"),
                      Text("⏱️ Time: ${telemetry!['timestamp']}"),
                    ],
                  ),
                ),
    );
  }
}
