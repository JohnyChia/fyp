import 'dart:convert';
import 'package:http/http.dart' as http;

class ApiService {
  static String _baseUrl(String ip) => 'http://$ip:5000';

  static Future<Map<String, dynamic>> getStatus(String ip) async {
  final url = '${_baseUrl(ip)}/api/status';

  print('[API DEBUG] GET $url');

  try {
    final res = await http
        .get(Uri.parse(url))
        .timeout(const Duration(seconds: 10));

    print('[API DEBUG] HTTP ${res.statusCode}');
    print('[API DEBUG] BODY: ${res.body}');

    return jsonDecode(res.body) as Map<String, dynamic>;
  } catch (e) {
    print('[API DEBUG] ERROR: $e');
    return {'error': e.toString()};
  }
}

  static Future<bool> sendCommand(String ip, String cmd) async {
    try {
      final res = await http
          .post(
            Uri.parse('${_baseUrl(ip)}/api/command'),
            headers: {'Content-Type': 'application/json'},
            body: jsonEncode({'cmd': cmd}),
          )
          .timeout(const Duration(seconds: 15));
      final data = jsonDecode(res.body) as Map<String, dynamic>;
      return data['success'] as bool? ?? false;
    } catch (_) {
      return false;
    }
  }

  static Future<List<dynamic>> getHistory(String ip) async {
    try {
      final res = await http
          .get(Uri.parse('${_baseUrl(ip)}/api/history'))
          .timeout(const Duration(seconds: 5));
      final data = jsonDecode(res.body) as Map<String, dynamic>;
      if (data['success'] as bool? ?? false) {
        return data['history'] as List<dynamic>? ?? [];
      }
      return [];
    } catch (_) {
      return [];
    }
  }
}
