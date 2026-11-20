# Sprint Plan (draft)

## Rules summary (Rulebook v1.0, Section 12.1)
- Lane 2.0 m x 7.0 m. Start line and finish line 7.0 m apart.
- 4 Gates, 60 cm openings, about every 1.4 m, alternating sides, taken in order.
- 2 Hurdles, 60 cm wide, 5 cm tall, on alternating sides. Cross over (going around costs -10).
- Clock: start signal to the Robot fully crossing the finish line. Run limit 2:30. No finish = 0.

## Score
`score = 100 - finish_time_s - 15*gates_missed - 10*hurdles_around - 15*resets (+10 if Gate 1 in Autonomous)`,
clamped to 0..100.

## Strategy notes
- +10 Autonomous bonus = 10 seconds. Autonomous Gate 1 is the cheapest points available.
- A missed Gate or a Reset costs 15 s equivalent. Clean and repeatable beats fast.
- Fully crossing the finish line is required, but leaving the Field voids the Run: stop just past it.
- Tether (if used) must stay slack. Prefer wireless.

## Open questions
- [ ] Finish detection method (odometry + ToF? no floor sensor in BOM).
- [ ] Encoders confirmed in drive BOM?
- [ ] Hurdle crossing torque (rated tractive force is about 24 N): bench test.
- [ ] Spline + Ramsete + EKF, or waypoints first?
