import 'dart:convert';
import 'package:http/http.dart' as http;

const relativeDetectionEvidenceLabel = 'Relative detection evidence';

double? relativeDetectionEvidencePercent(Map<String, dynamic> historyItem) {
  final value = historyItem['relative_detection_evidence_percent'] ??
      historyItem['coverage'];
  return value is num ? value.toDouble() : null;
}

String relativeDetectionEvidenceText(Map<String, dynamic> historyItem) {
  final value = relativeDetectionEvidencePercent(historyItem);
  return value == null ? 'N/A' : '${value.toStringAsFixed(1)}%';
}

class MissionStatusSnapshot {
  static const Set<String> _knownStates = {
    'READY',
    'INSPECTING',
    'RETURNING_HOME',
    'PROCESSING',
    'COMPLETED',
    'CANCELED',
    'FAILED',
  };
  static const Set<String> _activeStates = {
    'INSPECTING',
    'RETURNING_HOME',
    'PROCESSING',
  };
  static const Set<String> _terminalStates = {
    'COMPLETED',
    'CANCELED',
    'FAILED',
  };

  final String state;
  final String inspectionId;
  final bool finished;
  final bool succeeded;
  final String failureReason;

  const MissionStatusSnapshot({
    required this.state,
    required this.inspectionId,
    required this.finished,
    required this.succeeded,
    required this.failureReason,
  });

  factory MissionStatusSnapshot.fromJson(Map<String, dynamic> json) {
    final legacyFinished = json['inspection_finished'] as bool? ?? false;
    final rawState = json['inspection_state'] as String?;
    final state = rawState != null && _knownStates.contains(rawState)
        ? rawState
        : (legacyFinished ? 'COMPLETED' : 'UNKNOWN');

    return MissionStatusSnapshot(
      state: state,
      inspectionId: json['inspection_id'] as String? ?? '',
      finished: _terminalStates.contains(state),
      succeeded: state == 'COMPLETED' &&
          (json['inspection_succeeded'] as bool? ?? true),
      failureReason: json['failure_reason'] as String? ?? '',
    );
  }

  bool get isActive => _activeStates.contains(state);
  bool get isCompleted => state == 'COMPLETED';
  bool get isCanceled => state == 'CANCELED';
  bool get isFailed => state == 'FAILED';
}

class ApiService {
  static String _baseUrl(String ip) => 'http://$ip:5000';

  static Future<Map<String, dynamic>> getStatus(String ip) async {
    final url = '${_baseUrl(ip)}/api/status';

    print('[API DEBUG] GET $url');

    try {
      final res =
          await http.get(Uri.parse(url)).timeout(const Duration(seconds: 10));

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
