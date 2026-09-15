import 'dart:async';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'api_service.dart';
import 'video_stream_widget.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> with TickerProviderStateMixin {
  String _robotIp = '192.168.1.100';
  bool _serviceReady = false;
  bool _isInspecting = false;
  bool _inspectionFinished = false;
  String _statusText = 'Connecting...';
  Timer? _pollTimer;
  List<dynamic> _historyData = [];
  bool _loadingHistory = false;

  late AnimationController _pulseController;
  late AnimationController _bannerController;
  late Animation<double> _pulseAnim;
  late Animation<double> _bannerAnim;

  final TextEditingController _ipController = TextEditingController();
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();

  @override
  void initState() {
    super.initState();
    _pulseController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    )..repeat(reverse: true);
    _pulseAnim = Tween<double>(begin: 0.85, end: 1.0).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );

    _bannerController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 600),
    );
    _bannerAnim =
        CurvedAnimation(parent: _bannerController, curve: Curves.elasticOut);

    _loadIp();
  }

  Future<void> _loadIp() async {
    final prefs = await SharedPreferences.getInstance();
    final saved = prefs.getString('robot_ip') ?? '192.168.1.100';
    setState(() {
      _robotIp = saved;
      _ipController.text = saved;
    });
    _startPolling();
  }

  Future<void> _saveIp(String ip) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('robot_ip', ip);
    setState(() => _robotIp = ip);
    _pollTimer?.cancel();
    _startPolling();
  }

  void _startPolling() {
    _pollTimer?.cancel();
    _pollTimer =
        Timer.periodic(const Duration(seconds: 1), (_) => _pollStatus());
    _pollStatus();
  }

  Future<void> _pollStatus() async {
    try {
      final status = await ApiService.getStatus(_robotIp);
      if (!mounted) return;
      if (status.containsKey('error')) {
        setState(() {
          _serviceReady = false;
          // Show the exact error message so we can debug it on the phone
          _statusText = 'ERR: ${status['error']}';
        });
        return;
      }
      final finished = status['inspection_finished'] as bool? ?? false;
      setState(() {
        _serviceReady = status['service_ready'] as bool? ?? false;
        _statusText = status['current_status'] as String? ?? 'Running';
        _inspectionFinished = finished;
      });
      if (finished && !_bannerController.isCompleted) {
        _isInspecting = false;
        _bannerController.forward();
      } else if (!finished && _bannerController.isCompleted) {
        _bannerController.reverse();
      }
    } catch (_) {
      if (!mounted) return;
      setState(() {
        _serviceReady = false;
        _statusText = 'Connection Lost';
      });
    }
  }

  Future<void> _fetchHistory() async {
    setState(() => _loadingHistory = true);
    final data = await ApiService.getHistory(_robotIp);
    if (mounted) {
      setState(() {
        _historyData = data;
        _loadingHistory = false;
      });
    }
  }

  Future<void> _sendStart() async {
    if (!_serviceReady) {
      _showSnack('Service not ready, please wait for connection');
      return;
    }
    setState(() {
      _isInspecting = true;
      _inspectionFinished = false;
    });
    _bannerController.reverse();
    final ok = await ApiService.sendCommand(_robotIp, 'start');
    if (!ok && mounted) _showSnack('Failed to send command');
  }

  Future<void> _sendStop() async {
    setState(() => _isInspecting = false);
    final ok = await ApiService.sendCommand(_robotIp, 'stop');
    if (!ok && mounted) _showSnack('Failed to send stop command');
  }

  void _showSnack(String msg) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(msg),
        backgroundColor: const Color(0xFF1E293B),
        behavior: SnackBarBehavior.floating,
      ),
    );
  }

  void _showIpDialog() {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF1E293B),
        title:
            const Text('Set Robot IP', style: TextStyle(color: Colors.white)),
        content: TextField(
          controller: _ipController,
          style: const TextStyle(color: Colors.white),
          keyboardType: TextInputType.number,
          decoration: InputDecoration(
            hintText: 'e.g. 192.168.1.100',
            hintStyle: const TextStyle(color: Colors.white38),
            enabledBorder: OutlineInputBorder(
              borderSide:
                  BorderSide(color: const Color(0xFF4ADE80).withOpacity(0.4)),
              borderRadius: BorderRadius.circular(12),
            ),
            focusedBorder: OutlineInputBorder(
              borderSide: const BorderSide(color: Color(0xFF4ADE80)),
              borderRadius: BorderRadius.circular(12),
            ),
            prefixIcon: const Icon(Icons.wifi, color: Color(0xFF4ADE80)),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child:
                const Text('Cancel', style: TextStyle(color: Colors.white54)),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              backgroundColor: const Color(0xFF4ADE80),
              foregroundColor: Colors.black,
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(10)),
            ),
            onPressed: () {
              final ip = _ipController.text.trim();
              if (ip.isNotEmpty) _saveIp(ip);
              Navigator.pop(ctx);
            },
            child: const Text('Confirm'),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
    _pulseController.dispose();
    _bannerController.dispose();
    _ipController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: _scaffoldKey,
      drawer: _buildHistoryDrawer(),
      body: Container(
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [Color(0xFF0F172A), Color(0xFF1E293B)],
          ),
        ),
        child: SafeArea(
          child: Column(
            children: [
              _buildHeader(),
              Expanded(
                child: SingleChildScrollView(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Column(
                    children: [
                      const SizedBox(height: 8),
                      _buildVideoPanel(),
                      const SizedBox(height: 16),
                      _buildStatusCard(),
                      const SizedBox(height: 16),
                      _buildControls(),
                      const SizedBox(height: 16),
                      _buildFinishedBanner(),
                      const SizedBox(height: 24),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildHeader() {
    return Padding(
      padding: const EdgeInsets.fromLTRB(16, 16, 16, 8),
      child: Row(
        children: [
          IconButton(
            onPressed: () {
              _fetchHistory();
              _scaffoldKey.currentState?.openDrawer();
            },
            icon: const Icon(Icons.menu_rounded, color: Colors.white, size: 28),
          ),
          const SizedBox(width: 8),
          ShaderMask(
            shaderCallback: (bounds) => const LinearGradient(
              colors: [Color(0xFF4ADE80), Color(0xFF3B82F6)],
            ).createShader(bounds),
            child: const Text(
              'Durian AI',
              style: TextStyle(
                fontSize: 24,
                fontWeight: FontWeight.w800,
                color: Colors.white,
              ),
            ),
          ),
          const Spacer(),
          IconButton(
            onPressed: _showIpDialog,
            icon: const Icon(Icons.settings_remote, color: Color(0xFF94A3B8)),
            tooltip: 'Set Robot IP',
          ),
        ],
      ),
    );
  }

  Widget _buildHistoryDrawer() {
    return Drawer(
      backgroundColor: const Color(0xFF0F172A),
      child: SafeArea(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Padding(
              padding: const EdgeInsets.all(20),
              child: Row(
                children: [
                  const Icon(Icons.history_edu_rounded,
                      color: Color(0xFF4ADE80), size: 28),
                  const SizedBox(width: 12),
                  const Text(
                    'Inspection Logs',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 20,
                      fontWeight: FontWeight.w800,
                    ),
                  ),
                  const Spacer(),
                  IconButton(
                    icon: const Icon(Icons.refresh_rounded,
                        color: Colors.white70),
                    onPressed: _fetchHistory,
                  )
                ],
              ),
            ),
            const Divider(color: Colors.white12, height: 1),
            Expanded(
              child: _loadingHistory
                  ? const Center(
                      child: CircularProgressIndicator(
                        color: Color(0xFF4ADE80),
                      ),
                    )
                  : _historyData.isEmpty
                      ? const Center(
                          child: Text(
                            'No inspection records found',
                            style: TextStyle(color: Colors.white38),
                          ),
                        )
                      : ListView.builder(
                          padding: const EdgeInsets.symmetric(
                              horizontal: 12, vertical: 8),
                          itemCount: _historyData.length,
                          itemBuilder: (context, index) {
                            final item = _historyData[index];
                            final status = item['status'] ?? 'processing';
                            final isProcessing = status == 'processing';
                            final disease = isProcessing
                                ? 'Processing'
                                : (item['disease_name'] ?? 'Unknown');
                            final isHealthy = disease == 'Leaf_Healthy';
                            final color = isProcessing
                                ? const Color(0xFFF59E0B)
                                : (isHealthy
                                    ? const Color(0xFF4ADE80)
                                    : const Color(0xFFEF4444));

                            return Container(
                              margin: const EdgeInsets.only(bottom: 12),
                              padding: const EdgeInsets.all(14),
                              decoration: BoxDecoration(
                                color: const Color(0xFF1E293B),
                                borderRadius: BorderRadius.circular(16),
                                border: Border.all(
                                    color: Colors.white.withOpacity(0.05)),
                              ),
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  Row(
                                    mainAxisAlignment:
                                        MainAxisAlignment.spaceBetween,
                                    children: [
                                      Text(
                                        'Tree ID: ${item['tree_id'] ?? item['id']}',
                                        style: const TextStyle(
                                          color: Colors.white,
                                          fontWeight: FontWeight.w700,
                                          fontSize: 15,
                                        ),
                                      ),
                                      Container(
                                        padding: const EdgeInsets.symmetric(
                                            horizontal: 8, vertical: 4),
                                        decoration: BoxDecoration(
                                          color: color.withOpacity(0.15),
                                          borderRadius:
                                              BorderRadius.circular(8),
                                        ),
                                        child: Text(
                                          disease.replaceAll('Leaf_', ''),
                                          style: TextStyle(
                                            color: color,
                                            fontSize: 12,
                                            fontWeight: FontWeight.w600,
                                          ),
                                        ),
                                      ),
                                    ],
                                  ),
                                  const SizedBox(height: 8),
                                  _buildInfoRow('Coordinates',
                                      '(${item['tree_x']?.toStringAsFixed(2)}, ${item['tree_y']?.toStringAsFixed(2)})'),
                                  _buildInfoRow('Scan Duration',
                                      '${item['scan_time']?.toStringAsFixed(1)}s'),
                                  if (!isProcessing)
                                    _buildInfoRow('Severity Level',
                                        '${item['coverage']?.toStringAsFixed(1)}%'),
                                  if (!isProcessing && !isHealthy) ...[
                                    const SizedBox(height: 6),
                                    const Divider(color: Colors.white10),
                                    const SizedBox(height: 6),
                                    _buildInfoRow('Recommendation',
                                        item['remedy'] ?? 'None',
                                        labelColor: const Color(0xFF3B82F6)),
                                    _buildInfoRow('Priority Rank',
                                        item['priority'] ?? 'None',
                                        labelColor: const Color(0xFFF59E0B)),
                                  ],
                                  const SizedBox(height: 6),
                                  Text(
                                    item['timestamp'] ?? '',
                                    style: const TextStyle(
                                        color: Colors.white24, fontSize: 10),
                                  ),
                                ],
                              ),
                            );
                          },
                        ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildInfoRow(String label, String value, {Color? labelColor}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label,
              style: const TextStyle(color: Colors.white54, fontSize: 13)),
          Text(
            value,
            style: TextStyle(
              color: labelColor ?? Colors.white70,
              fontSize: 13,
              fontWeight:
                  labelColor != null ? FontWeight.w600 : FontWeight.normal,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildVideoPanel() {
    return Container(
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(24),
        gradient: LinearGradient(
          colors: [
            Colors.white.withOpacity(0.08),
            Colors.white.withOpacity(0.03),
          ],
        ),
        border: Border.all(color: Colors.white.withOpacity(0.1)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.4),
            blurRadius: 30,
            offset: const Offset(0, 8),
          )
        ],
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(24),
        child: Stack(
          children: [
            AspectRatio(
              aspectRatio: 16 / 9,
              child: VideoStreamWidget(robotIp: _robotIp),
            ),
            Positioned(
              top: 12,
              right: 12,
              child: _buildStatusBadge(),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusBadge() {
    final color =
        _serviceReady ? const Color(0xFF4ADE80) : const Color(0xFFEF4444);
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.black.withOpacity(0.6),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.white.withOpacity(0.1)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          ScaleTransition(
            scale: _pulseAnim,
            child: Container(
              width: 8,
              height: 8,
              decoration: BoxDecoration(
                color: color,
                shape: BoxShape.circle,
                boxShadow: [BoxShadow(color: color, blurRadius: 6)],
              ),
            ),
          ),
          const SizedBox(width: 8),
          Text(
            _statusText,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 12,
              fontWeight: FontWeight.w600,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusCard() {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        color: Colors.white.withOpacity(0.05),
        border: Border.all(color: Colors.white.withOpacity(0.08)),
      ),
      child: Row(
        children: [
          Icon(
            _serviceReady ? Icons.link : Icons.link_off,
            color: _serviceReady
                ? const Color(0xFF4ADE80)
                : const Color(0xFFEF4444),
            size: 20,
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Text(
              'Robot IP: $_robotIp',
              style: const TextStyle(color: Color(0xFF94A3B8), fontSize: 13),
            ),
          ),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
            decoration: BoxDecoration(
              color: (_serviceReady
                      ? const Color(0xFF4ADE80)
                      : const Color(0xFFEF4444))
                  .withOpacity(0.15),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Text(
              _serviceReady ? 'Connected' : 'Waiting...',
              style: TextStyle(
                color: _serviceReady
                    ? const Color(0xFF4ADE80)
                    : const Color(0xFFEF4444),
                fontSize: 12,
                fontWeight: FontWeight.w600,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildControls() {
    return Row(
      children: [
        Expanded(
          child: _ActionButton(
            label: 'Start Inspection',
            icon: Icons.play_arrow_rounded,
            gradient: const LinearGradient(
              colors: [Color(0xFF10B981), Color(0xFF059669)],
            ),
            shadowColor: const Color(0xFF10B981),
            onTap: (_isInspecting || !_serviceReady) ? null : _sendStart,
          ),
        ),
        const SizedBox(width: 12),
        Expanded(
          child: _ActionButton(
            label: 'Return Home',
            icon: Icons.home_rounded,
            gradient: const LinearGradient(
              colors: [Color(0xFFEF4444), Color(0xFFDC2626)],
            ),
            shadowColor: const Color(0xFFEF4444),
            onTap: !_serviceReady ? null : _sendStop,
          ),
        ),
      ],
    );
  }

  Widget _buildFinishedBanner() {
    return ScaleTransition(
      scale: _bannerAnim,
      child: _inspectionFinished
          ? Container(
              width: double.infinity,
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(20),
                gradient: LinearGradient(
                  colors: [
                    const Color(0xFF4ADE80).withOpacity(0.2),
                    const Color(0xFF3B82F6).withOpacity(0.1),
                  ],
                ),
                border:
                    Border.all(color: const Color(0xFF4ADE80).withOpacity(0.4)),
              ),
              child: const Column(
                children: [
                  Text('✅', style: TextStyle(fontSize: 40)),
                  SizedBox(height: 8),
                  Text(
                    'Inspection Complete!',
                    style: TextStyle(
                      color: Color(0xFF4ADE80),
                      fontSize: 20,
                      fontWeight: FontWeight.w700,
                    ),
                  ),
                  SizedBox(height: 4),
                  Text(
                    'Robot has safely returned to the initial point',
                    style: TextStyle(color: Color(0xFF94A3B8), fontSize: 14),
                  ),
                ],
              ),
            )
          : const SizedBox.shrink(),
    );
  }
}

class _ActionButton extends StatefulWidget {
  final String label;
  final IconData icon;
  final LinearGradient gradient;
  final Color shadowColor;
  final VoidCallback? onTap;

  const _ActionButton({
    required this.label,
    required this.icon,
    required this.gradient,
    required this.shadowColor,
    this.onTap,
  });

  @override
  State<_ActionButton> createState() => _ActionButtonState();
}

class _ActionButtonState extends State<_ActionButton>
    with SingleTickerProviderStateMixin {
  late AnimationController _ctrl;
  late Animation<double> _scale;

  @override
  void initState() {
    super.initState();
    _ctrl = AnimationController(
        vsync: this, duration: const Duration(milliseconds: 120));
    _scale = Tween<double>(begin: 1.0, end: 0.95).animate(
      CurvedAnimation(parent: _ctrl, curve: Curves.easeOut),
    );
  }

  @override
  void dispose() {
    _ctrl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final disabled = widget.onTap == null;
    return GestureDetector(
      onTapDown: disabled ? null : (_) => _ctrl.forward(),
      onTapUp: disabled
          ? null
          : (_) {
              _ctrl.reverse();
              widget.onTap?.call();
            },
      onTapCancel: () => _ctrl.reverse(),
      child: ScaleTransition(
        scale: _scale,
        child: Opacity(
          opacity: disabled ? 0.45 : 1.0,
          child: Container(
            padding: const EdgeInsets.symmetric(vertical: 18),
            decoration: BoxDecoration(
              gradient: widget.gradient,
              borderRadius: BorderRadius.circular(18),
              boxShadow: disabled
                  ? []
                  : [
                      BoxShadow(
                        color: widget.shadowColor.withOpacity(0.35),
                        blurRadius: 16,
                        offset: const Offset(0, 6),
                      )
                    ],
            ),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(widget.icon, color: Colors.white, size: 22),
                const SizedBox(width: 8),
                Text(
                  widget.label,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 16,
                    fontWeight: FontWeight.w700,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
