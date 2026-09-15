import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;

class VideoStreamWidget extends StatefulWidget {
  final String robotIp;

  const VideoStreamWidget({super.key, required this.robotIp});

  @override
  State<VideoStreamWidget> createState() => _VideoStreamWidgetState();
}

class _VideoStreamWidgetState extends State<VideoStreamWidget> {
  Uint8List? _currentFrame;
  bool _isConnected = false;
  bool _hasError = false;
  StreamSubscription? _sub;
  http.Client? _client;

  @override
  void initState() {
    super.initState();
    _connect();
  }

  @override
  void didUpdateWidget(VideoStreamWidget old) {
    super.didUpdateWidget(old);
    if (old.robotIp != widget.robotIp) {
      _disconnect();
      _connect();
    }
  }

  Future<void> _connect() async {
    _disconnect();
    setState(() {
      _hasError = false;
      _isConnected = false;
    });

    try {
      _client = http.Client();
      final request = http.Request(
        'GET',
        Uri.parse('http://${widget.robotIp}:5000/video_feed'),
      );
      request.headers['Connection'] = 'keep-alive';

      final response = await _client!
          .send(request)
          .timeout(const Duration(seconds: 5));

      if (response.statusCode != 200) {
        _onError();
        return;
      }

      setState(() => _isConnected = true);

      final buffer = <int>[];
      _sub = response.stream.listen(
        (chunk) {
          buffer.addAll(chunk);
          _extractFrames(buffer);
        },
        onError: (_) => _onError(),
        onDone: () {
          if (mounted) {
            Future.delayed(const Duration(seconds: 2), _connect);
          }
        },
        cancelOnError: true,
      );
    } catch (_) {
      _onError();
    }
  }

  void _extractFrames(List<int> buffer) {
    // 找到最后一个完整帧，丢弃所有中间帧以减少延迟
    int lastStart = -1;
    int lastEnd = -1;
    
    for (int i = 0; i < buffer.length - 1; i++) {
      if (buffer[i] == 0xFF && buffer[i + 1] == 0xD8) {
        lastStart = i;
      }
      if (lastStart != -1 && buffer[i] == 0xFF && buffer[i + 1] == 0xD9) {
        lastEnd = i + 2;
      }
    }
    
    if (lastStart != -1 && lastEnd != -1 && lastEnd > lastStart) {
      final frameBytes = Uint8List.fromList(buffer.sublist(lastStart, lastEnd));
      buffer.removeRange(0, lastEnd);
      if (mounted) {
        setState(() => _currentFrame = frameBytes);
      }
    }
    
    // 防止缓冲区无限增长
    if (buffer.length > 200000) buffer.removeRange(0, buffer.length - 50000);
  }

  void _onError() {
    if (!mounted) return;
    setState(() {
      _hasError = true;
      _isConnected = false;
    });
    Future.delayed(const Duration(seconds: 3), () {
      if (mounted) _connect();
    });
  }

  void _disconnect() {
    _sub?.cancel();
    _sub = null;
    _client?.close();
    _client = null;
  }

  @override
  void dispose() {
    _disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (_currentFrame != null) {
      return Image.memory(
        _currentFrame!,
        gaplessPlayback: true,
        fit: BoxFit.cover,
        width: double.infinity,
        height: double.infinity,
      );
    }

    return Container(
      color: const Color(0xFF0F172A),
      child: Center(
        child: _hasError
            ? const Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(Icons.videocam_off_rounded,
                      color: Color(0xFF475569), size: 48),
                  SizedBox(height: 12),
                  Text(
                    'Camera Offline',
                    style: TextStyle(color: Color(0xFF475569), fontSize: 16),
                  ),
                ],
              )
            : const Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  CircularProgressIndicator(color: Color(0xFF4ADE80)),
                  SizedBox(height: 12),
                  Text(
                    'Connecting to camera...',
                    style: TextStyle(color: Color(0xFF94A3B8), fontSize: 14),
                  ),
                ],
              ),
      ),
    );
  }
}
