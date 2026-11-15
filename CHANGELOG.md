# Changelog

## 2026.11.15.2

- **Rocket boosters.** Booster weight follows the game: JATO/RATO pods bring their full propellant (Kitsuka 700 kg, Me 264 900, Tu-14T 760, B-66B 408, Il-28 380, Ar 234 250, Pucara 120, A-4 81), the Mirage SEPR store both of its tanks (1,080 kg), and the Me 262 C-1a/C-2b their full 1,700 kg of rocket propellant whatever the fuel load. Boosters the aircraft always carries (Kitsuka, A-4, Ar 234, B-66B, Il-28, Me 264, Tu-14T) are part of the default.
- **Booster on** (next to the throttle, EM and engine pages, shown only when a booster is fitted) lights the boosters as the game's ignite key does: full rated thrust at any speed and altitude, independent of the throttle, until the propellant runs out. The point details give the thrust, propellant and burn time.
- **Booster propellant** (next to the fuel load): 100% is the full load the game spawns with; less scales the booster propellant and its burn time; 0% is burnt out, when auto-dropping booster pods have left with their weight and drag.
- **Mistel 2.** With the Mistel loadout the chart flies the Ju 88 G-1 Mistel's own flight model, mass, fuel, engines and Instructor, as the game does while the two are joined; the Fw 190 adds no weight, drag or thrust there. With the stores released (or no loadout) it is the clean Fw 190 A-8. It was a Fw 190 with a 3,500 kg zero-drag bomb before.

## 2026.11.15.1

- With **Manual engine controls** on, every Fw 190 and Ta 152, the Bf 109 E-3 (Japan)/E-4/E-7/T-2/Z, Bf 110 C-4/C-6/F-2, Do 335 A-0/A-1/B-2, Me 210, Ju 388 J and Bv 155 B-1 offer **Automatic propeller control** again, on by default as in the game. The game allows automatic pitch on every blade-angle governor of this kind, whatever the FM's HasAutoPitchControl says; the calculator used that flag and forced a manual blade angle. Charts with manual engine controls off are unchanged.
- Copying settings from an aircraft without automatic pitch (for example a P-51) no longer turns automatic pitch off for the other entries, on the EM and engine pages. The engine page also updates the propeller setting when an entry changes game version.

## 2026.11.14.1

- The flight-model calculators accept altitudes up to 30,000 m (was 18,000 m) and true airspeeds up to 4,000 km/h (was 2,600 km/h). This applies to the EM chart, the altitude envelope, the engine page and clicked points. In the game's atmosphere the air thins only slowly above 18.3 km. The MiG-25PD's 1 g ceiling is about 25 km and its Mach limit is 3,240 km/h, and the F-15C, Su-27, Lightning and F-104S ceilings lie between 18.1 and 19.8 km. Default ranges are unchanged.
- The engine page now finds level flight that begins only at high speed, as at high altitude or past VNE at a fixed speed. If none of its usual starting points converges, it tries points spread over the requested range.
- Charts over the wider ranges put grid lines every 200 km/h and 2,000 m.

## 2026.11.11.5

- All FM versions use one shared HTTP service and the same compiled solver. Version inputs and catalogs are prepared ahead of use, with a bounded pool of warm calculation workers.
- Short calculations reach the final chart without downloading an unnecessary preview first. Completed charts and point details survive worker reuse and input updates.
- Newly built inputs become available after verification; fresh pages continue to default to the newest available version.

## 2026.11.11.4

- Short-release aircraft (release start at most 10 seconds) use the released Instructor pull boundary everywhere, without reapplying the wing-protection cap when the in-game release conditions are absent. Other limits and longer-release aircraft retain their existing behavior.


## 2026.11.11.3

- Aircraft added or duplicated in Engine Performance now keep valid, unique entry IDs when switching to EM or altitude diagrams, so those calculations accept the shared aircraft list.

## 2026.11.11.2

- Fresh pages default to the newest available game version. EM, engine, and missile calculators load that version's catalog and send calculations to its server.
- Manually selected historical versions keep their own data and calculation routes. A default version's server starts again when needed.
