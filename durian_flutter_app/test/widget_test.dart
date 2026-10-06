import 'package:durian_inspection_app/api_service.dart';
import 'package:durian_inspection_app/home_screen.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('history evidence uses the explicit field with legacy coverage fallback',
      () {
    expect(relativeDetectionEvidenceLabel, 'Relative detection evidence');
    expect(
      relativeDetectionEvidencePercent({
        'relative_detection_evidence_percent': 72.5,
        'coverage': 1.0,
      }),
      72.5,
    );
    expect(relativeDetectionEvidencePercent({'coverage': 41}), 41.0);
    expect(relativeDetectionEvidenceText({'coverage': 41}), '41.0%');
    expect(relativeDetectionEvidenceText({}), 'N/A');
  });

  test('ready mission remains idle and non-terminal', () {
    final status = MissionStatusSnapshot.fromJson({
      'inspection_state': 'READY',
      'inspection_id': '',
      'inspection_finished': false,
      'inspection_succeeded': false,
      'failure_reason': '',
    });
    expect(status.state, 'READY');
    expect(status.isActive, isFalse);
    expect(status.finished, isFalse);
    expect(status.succeeded, isFalse);
  });

  test('active mission states remain active', () {
    for (final state in ['INSPECTING', 'RETURNING_HOME', 'PROCESSING']) {
      final status = MissionStatusSnapshot.fromJson({
        'inspection_state': state,
        'inspection_id': 'inspection-42',
        'inspection_finished': false,
        'inspection_succeeded': false,
        'failure_reason': '',
      });
      expect(status.isActive, isTrue);
      expect(status.finished, isFalse);
    }
  });

  testWidgets('completed mission shows completion UI', (tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: MissionTerminalBanner(
            state: 'COMPLETED',
            succeeded: true,
            failureReason: '',
          ),
        ),
      ),
    );

    expect(find.text('Inspection Complete!'), findsOneWidget);
    expect(find.text('Inspection Canceled'), findsNothing);
    expect(find.text('Inspection Failed'), findsNothing);
  });

  testWidgets('canceled mission is not shown as success', (tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: MissionTerminalBanner(
            state: 'CANCELED',
            succeeded: false,
            failureReason: 'cancel_requested',
          ),
        ),
      ),
    );

    expect(find.text('Inspection Canceled'), findsOneWidget);
    expect(find.text('Inspection Complete!'), findsNothing);
  });

  testWidgets('failed mission shows the failure reason', (tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: MissionTerminalBanner(
            state: 'FAILED',
            succeeded: false,
            failureReason: 'model unavailable',
          ),
        ),
      ),
    );

    expect(find.text('Inspection Failed'), findsOneWidget);
    expect(find.text('model unavailable'), findsOneWidget);
    expect(find.text('Inspection Complete!'), findsNothing);
  });
}
