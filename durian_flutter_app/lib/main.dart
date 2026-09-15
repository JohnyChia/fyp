import 'package:flutter/material.dart';
import 'home_screen.dart';

void main() {
  runApp(const DurianInspectionApp());
}

class DurianInspectionApp extends StatelessWidget {
  const DurianInspectionApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Durian AI Inspection',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: const ColorScheme.dark(
          primary: Color(0xFF4ADE80),
          secondary: Color(0xFF3B82F6),
          surface: Color(0xFF1E293B),
          onPrimary: Colors.white,
        ),
        scaffoldBackgroundColor: const Color(0xFF0F172A),
        fontFamily: 'Roboto',
        useMaterial3: true,
      ),
      home: const HomeScreen(),
    );
  }
}
