import 'dart:convert';
import 'package:http/http.dart' as http;

class ApiService {
  static const String baseUrl = "http://10.0.2.2:8000"; // for Android emulator
  // Use http://localhost:8000 for Flutter web

  static Future<List<dynamic>> getTelemetry() async {
    final response = await http.get(Uri.parse("$baseUrl/api/telemetry/latest"));
    if (response.statusCode == 200) {
      return jsonDecode(response.body);
    } else {
      throw Exception("Failed to load telemetry");
    }
  }
}
