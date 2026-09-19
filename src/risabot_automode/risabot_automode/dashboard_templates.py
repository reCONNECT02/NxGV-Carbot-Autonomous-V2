# dashboard_templates.py
# Auto-extracted HTML templates for the RISA-bot dashboard.
# Edit these to change the dashboard UI appearance.

# ======================== Main HTML Dashboard ========================
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RISA-Bot Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&display=swap" rel="stylesheet">
<style>
  :root {
    /* Catppuccin Latte (Light Theme) */
    --bg: #eff1f5;
    --surface: #e6e9ef;
    --surface2: #ccd0da;
    --text: #4c4f69;
    --accent: #1e66f5; /* Sapphire */
    --accent2: rgba(30, 102, 245, 0.12);
    --danger: #d20f39; /* Red */
    --success: #40a02b; /* Green */
    --warning: #df8e1d; /* Yellow */
    --muted: #8c8fa1;
    --card: #e6e9ef;
    --card-border: rgba(0, 0, 0, 0.06);
    --radius: 16px;
    --glow: rgba(30, 102, 245, 0.15);
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Inter', system-ui, -apple-system, sans-serif;
    font-size: 18px; /* Bigger base size for readability */
    font-weight: 500;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    overflow-x: hidden;
    position: relative;
  }

  /* ===== ANIMATED ORB BACKGROUND ===== */
  .bg-orbs {
    position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
    z-index: -1; overflow: hidden; pointer-events: none;
  }
  .orb {
    position: absolute; border-radius: 50%; filter: blur(60px);
    animation: float 20s infinite ease-in-out alternate; opacity: 0.15;
  }
  .orb-1 { width: 400px; height: 400px; background: rgba(233,69,96,1); top: -10%; left: 10%; animation-delay: 0s; }
  .orb-2 { width: 500px; height: 500px; background: rgba(15,52,96,1); bottom: -20%; right: -10%; animation-delay: -5s; }
  .orb-3 { width: 300px; height: 300px; background: rgba(66,165,245,1); top: 50%; left: 60%; animation-delay: -10s; }
  @keyframes float { 0% { transform: translateY(0) scale(1); } 100% { transform: translateY(-50px) scale(1.1); } }

  /* ===== WARNING FLASH OVERLAY ===== */
  .warning-overlay {
    position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
    box-shadow: inset 0 0 100px rgba(255,0,0,0.3);
    pointer-events: none; opacity: 0; z-index: 999;
    transition: opacity 0.3s;
  }
  .warning-overlay.active { animation: flashDanger 1.5s infinite alternate; }
  @keyframes flashDanger { 0% { opacity: 0.1; } 100% { opacity: 0.4; } }

  /* ===== HEADER ===== */
  .header {
    background: #fff;
    padding: 14px 28px;
    border-bottom: 1px solid rgba(0,0,0,0.08);
    display: flex;
    align-items: center;
    justify-content: space-between;
    position: sticky;
    top: 0;
    z-index: 100;
    box-shadow: 0 2px 12px rgba(0,0,0,0.06);
  }
  .header h1 {
    font-size: 1.4em;
    font-weight: 800;
    background: linear-gradient(135deg, var(--accent), #ff6b81, #42a5f5);
    background-size: 200% 200%;
    animation: gradShift 4s ease infinite;
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    letter-spacing: -0.5px;
  }
  @keyframes gradShift { 0%,100%{background-position:0% 50%} 50%{background-position:100% 50%} }
  .conn-badge {
    display: flex;
    align-items: center;
    gap: 10px;
    font-size: 0.8em;
    color: var(--muted);
  }
  .conn-dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: #4caf50;
    animation: pulse 2s infinite;
  }
  .header-meta {
    font-size: 0.72em;
    color: var(--muted);
    opacity: 0.7;
    font-weight: 500;
    font-variant-numeric: tabular-nums;
  }
  .latency-badge {
    font-size: 0.65em;
    padding: 2px 6px;
    border-radius: 4px;
    background: rgba(255,255,255,0.05);
    color: var(--muted);
    font-weight: 600;
    font-variant-numeric: tabular-nums;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }

  /* ===== LAYOUT ===== */
  .layout {
    display: grid;
    grid-template-columns: 260px 1fr 260px;
    gap: 14px;
    padding: 14px;
    max-width: 1600px;
    margin: 0 auto;
    transition: transform 0.4s cubic-bezier(0.4,0,0.2,1);
  }
  .layout.drawer-open {
    transform: translateX(100px);
  }
  .col { display: flex; flex-direction: column; gap: 12px; }

  /* ===== CARDS ===== */
  .card {
    background: var(--surface);
    border-radius: var(--radius);
    padding: 20px;
    box-shadow: 0 8px 24px rgba(0,0,0,0.05);
    border: 1px solid rgba(0,0,0,0.05);
  }
  .card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0;
    transition: opacity 0.3s;
  }
  .card:hover {
    border-color: rgba(233,69,96,0.3);
    box-shadow: 0 15px 35px rgba(233,69,96,0.15), inset 0 0 20px rgba(255,255,255,0.02);
    transform: translateY(-4px) scale(1.02);
  }
  .card:hover::before { opacity: 1; }
  .card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 12px;
    font-weight: 700;
  }

  /* ===== STATE BADGE ===== */
  .state-badge {
    display: inline-block;
    padding: 6px 14px;
    border-radius: 8px;
    font-size: 1em;
    font-weight: 700;
    letter-spacing: 0.5px;
    transition: all 0.3s;
  }
  .state-LANE_FOLLOW { background: #1b5e20; color: #a5d6a7; }
  .state-OBSTRUCTION { background: #e65100; color: #ffcc80; }
  .state-ROUNDABOUT { background: #4a148c; color: #ce93d8; }
  .state-BOOM_GATE_1,.state-BOOM_GATE_2 { background: #b71c1c; color: #ef9a9a; }
  .state-TUNNEL { background: #263238; color: #90a4ae; }
  .state-HILL { background: #33691e; color: #aed581; }
  .state-BUMPER { background: #795548; color: #d7ccc8; }
  .state-TRAFFIC_LIGHT { background: #f57f17; color: #fff9c4; }
  .state-PARALLEL_PARK,.state-PERPENDICULAR_PARK { background: #0d47a1; color: #90caf9; }
  .state-DRIVE_TO_PERP { background: #1565c0; color: #90caf9; }
  .state-EMERGENCY_STOP { background: #b71c1c; color: #ffcdd2; }
  .state-FINISHED { background: #ffd600; color: #333; }

  .mode-badge {
    padding: 4px 12px;
    border-radius: 6px;
    font-weight: 700;
    font-size: 0.75em;
    letter-spacing: 1px;
    transition: all 0.3s;
    vertical-align: middle;
  }
  .mode-AUTO { background: #4caf50; color: #fff; box-shadow: 0 0 8px rgba(76,175,80,0.3); }
  .mode-MANUAL { background: #d32f2f; color: #fff; box-shadow: 0 0 8px rgba(211,47,47,0.3); }

  .state-action-btn:hover {
    transform: translateY(-1px);
    box-shadow: 0 4px 12px rgba(0,0,0,0.1);
  }
  .state-action-btn:active {
    transform: translateY(0);
  }
  .state-action-btn.reset:hover { background: rgba(223, 142, 29, 0.18) !important; border-color: var(--warning) !important; }
  .state-action-btn.lap1:hover { background: rgba(30, 102, 245, 0.18) !important; border-color: var(--accent) !important; }
  .state-action-btn.lap2:hover { background: rgba(66, 165, 245, 0.18) !important; border-color: #42a5f5 !important; }

  /* ===== RECORD & PLAYBACK ===== */
  .rp-state-badge {
    display: inline-block;
    padding: 3px 8px;
    border-radius: 6px;
    font-size: 0.9em;
    font-weight: 800;
    letter-spacing: 0.5px;
    transition: all 0.4s;
  }
  .rp-state-badge.idle {
    background: rgba(108,112,134,0.15);
    color: var(--text);
    border: 1px solid rgba(108,112,134,0.3);
  }
  .rp-state-badge.recording {
    background: rgba(255,82,82,0.15);
    color: var(--danger);
    border: 1px solid rgba(255,82,82,0.4);
    animation: recPulse 1.5s ease infinite;
  }
  .rp-state-badge.playback {
    background: rgba(105,240,174,0.15);
    color: var(--success);
    border: 1px solid rgba(105,240,174,0.4);
    animation: playPulse 1.5s ease infinite;
  }
  @keyframes recPulse { 0%,100%{box-shadow:0 0 0 0 rgba(255,82,82,0.3)} 50%{box-shadow:0 0 20px 4px rgba(255,82,82,0.2)} }
  @keyframes playPulse { 0%,100%{box-shadow:0 0 0 0 rgba(105,240,174,0.3)} 50%{box-shadow:0 0 20px 4px rgba(105,240,174,0.2)} }

  .rp-btn.record.active {
    background: var(--danger) !important;
    color: #fff !important;
    border-color: var(--danger) !important;
    box-shadow: 0 0 16px rgba(210,15,57,0.4);
  }
  .rp-btn.play.active {
    background: var(--success) !important;
    color: #fff !important;
    border-color: var(--success) !important;
    box-shadow: 0 0 16px rgba(64,160,43,0.4);
  }

  .lap-badge {
    padding: 4px 10px;
    border-radius: 6px;
    font-weight: 600;
    font-size: 0.8em;
    background: var(--accent2);
    color: var(--accent);
  }
  .info-row {
    display: flex;
    gap: 16px;
    margin-top: 8px;
    font-size: 0.75em;
    color: var(--muted);
  }

  /* ===== TRAFFIC LIGHT ===== */
  .traffic-light {
    display: flex; gap: 12px; align-items: center; justify-content: center;
    background: linear-gradient(145deg, #555, #444);
    padding: 14px 24px; border-radius: 40px;
    width: fit-content; margin: 0 auto;
    box-shadow: inset 0 2px 6px rgba(0,0,0,0.3), 0 2px 8px rgba(0,0,0,0.1);
    border: 1px solid rgba(0,0,0,0.1);
  }
  .tl-circle {
    width: 32px; height: 32px; border-radius: 50%;
    border: 2px solid #000; transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
    box-shadow: inset 0 2px 6px rgba(0,0,0,0.8);
  }
  .tl-red { background: #3a1111; }
  .tl-yellow { background: #3a3311; }
  .tl-green { background: #113a11; }

  .tl-red.active { 
    background: radial-gradient(circle at 30% 30%, #ff8a80, #f44336); 
    box-shadow: 0 0 20px #f44336, 0 0 40px rgba(244,67,54,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #ff5252;
  }
  .tl-yellow.active { 
    background: radial-gradient(circle at 30% 30%, #fff59d, #ffeb3b); 
    box-shadow: 0 0 20px #ffeb3b, 0 0 40px rgba(255,235,59,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #ffff00;
  }
  .tl-green.active { 
    background: radial-gradient(circle at 30% 30%, #a5d6a7, #4caf50); 
    box-shadow: 0 0 20px #4caf50, 0 0 40px rgba(76,175,80,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #69f0ae;
  }
  .tl-label { text-align: center; margin-top: 8px; font-size: 0.75em; color: var(--muted); font-weight: 600; text-transform: uppercase; letter-spacing: 1px; }

  /* ===== SENSOR ROWS ===== */
  .s-row {
    display: flex;
    align-items: center;
    padding: 5px 0;
    border-bottom: 1px solid rgba(255,255,255,0.03);
  }
  .s-row:last-child { border: none; }
  .s-label { flex: 1; font-size: 0.78em; color: #999; }
  .s-val { font-weight: 600; font-size: 0.82em; }
  .dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    display: inline-block;
    margin-right: 5px;
    transition: all 0.3s;
  }
  .dot-green { background: #4caf50; box-shadow: 0 0 6px #4caf50; }
  .dot-red { background: #f44336; box-shadow: 0 0 6px #f44336; }
  .dot-gray { background: #333; }

  /* ===== METER BARS ===== */
  .meter { height: 8px; background: rgba(26,26,46,0.8); border-radius: 4px; overflow: hidden; margin-top: 6px; box-shadow: inset 0 1px 3px rgba(0,0,0,0.3); }
  .meter-fill { height: 100%; border-radius: 4px; transition: width 0.4s cubic-bezier(0.4,0,0.2,1), background 0.4s; position: relative; }
  .meter-fill::after { content: ''; position: absolute; top: 0; left: 0; right: 0; height: 50%; background: linear-gradient(180deg, rgba(255,255,255,0.15), transparent); border-radius: 4px 4px 0 0; }
  .meter-blue { background: linear-gradient(90deg, #1565c0, #42a5f5); }
  .meter-orange { background: linear-gradient(90deg, #e65100, #ff9800, #ffb74d); }

  /* ===== SPEED GAUGE ===== */
  .speed-display {
    text-align: center;
    padding: 12px 0;
  }
  .speed-display .num {
    font-size: 2.8em;
    font-weight: 800;
    background: linear-gradient(135deg, #42a5f5, #1e88e5, #e94560);
    background-size: 200% 200%;
    animation: gradShift 3s ease infinite;
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    line-height: 1;
    transition: all 0.3s;
  }
  .speed-display .unit {
    font-size: 0.6em;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 3px;
    margin-top: 4px;
  }
  .gear-dots {
    display: flex; justify-content: center; gap: 6px; margin-top: 8px;
  }
  .gear-dot {
    width: 12px; height: 12px; border-radius: 50%; background: rgba(255,255,255,0.08);
    border: 1px solid rgba(255,255,255,0.1);
    transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
  }
  .gear-dot.active {
    background: var(--accent); border-color: #fff;
    box-shadow: 0 0 15px rgba(233,69,96,0.8), 0 0 30px rgba(233,69,96,0.4);
    transform: scale(1.3);
  }

  /* ===== CHALLENGE SELECTOR ===== */
  .ctrl-state-badge {
    display: inline-block;
    padding: 4px 10px;
    border-radius: 6px;
    font-weight: 700;
    font-size: 0.78em;
    background: #4a148c;
    color: #fff;
    border: 1px solid rgba(206,147,216,0.2);
  }

  /* ===== CAMERA ===== */
  .cam-card {
    display: flex;
    flex-direction: column;
  }
  .cam-toggle {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 8px 16px;
    border-radius: 8px;
    border: 1px solid rgba(255,255,255,0.1);
    background: rgba(255,255,255,0.05);
    color: #42a5f5;
    cursor: pointer;
    font-size: 0.8em;
    font-weight: 600;
    transition: all 0.2s;
    margin-bottom: 10px;
    font-family: inherit;
  }
  .cam-toggle:hover { background: rgba(66,165,245,0.15); border-color: #42a5f5; }
  .cam-toggle.on { background: rgba(76,175,80,0.2); color: #4caf50; border-color: #4caf50; }
  .cam-container {
    width: 100%;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #050508;
    border-radius: 12px;
    overflow: hidden;
    position: relative;
  }
  .cam-container img {
    width: 100%;
    height: 100%;
    object-fit: contain;
    border-radius: 8px;
  }
  .cam-container.active { border-color: rgba(66,165,245,0.4); box-shadow: 0 0 20px rgba(66,165,245,0.1) inset; }
  .cam-off { display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; color: #555; font-size: 0.85em; }
  .cam-off .icon { font-size: 2em; margin-bottom: 8px; opacity: 0.5; }
  .cam-tabs { display: flex; gap: 4px; background: rgba(0,0,0,0.05); padding: 4px; border-radius: 8px; margin-bottom: 8px; }
  .cam-tab { flex: 1; text-align: center; font-size: 0.7em; padding: 6px 0; border-radius: 6px; cursor: pointer; color: #5c5f77; transition: all 0.2s; font-weight: 600; }
  .cam-tab:hover { background: rgba(0,0,0,0.05); color: #4c4f69; }
  .cam-tab.active { background: var(--accent); color: #fff; }
  
  /* ===== FLOW TIMELINE ===== */
  .ctrl-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 4px;
    text-align: center;
  }
  .ctrl-btn {
    padding: 6px 2px;
    border-radius: 6px;
    background: rgba(0,0,0,0.04);
    border: 1px solid rgba(0,0,0,0.06);
    font-size: 0.7em;
    color: #6c6f85;
    transition: all 0.15s;
    font-weight: 600;
  }
  .ctrl-btn.active {
    background: var(--accent);
    color: #fff;
    border-color: var(--accent);
    box-shadow: 0 0 8px rgba(30,102,245,0.3);
  }
  .joy-info {
    margin-top: 14px;
    display: flex;
    justify-content: center;
    gap: 30px;
  }
  .joy-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
  }
  .joy-circle {
    width: 60px; height: 60px;
    border-radius: 50%;
    background: #dce0e8;
    border: 1px solid rgba(0,0,0,0.1);
    position: relative;
  }
  .joy-dot {
    width: 14px; height: 14px;
    background: var(--muted);
    border-radius: 50%;
    position: absolute;
    top: 50%; left: 50%;
    transform: translate(-50%, -50%);
    box-shadow: 0 0 8px rgba(0,0,0,0.1);
    transition: background 0.2s;
  }
  .joy-dot.active {
    background: var(--accent);
    box-shadow: 0 0 12px var(--accent);
  }
  .joy-label {
    font-size: 0.65em;
    color: var(--muted);
    font-weight: 600;
  }
  .btn-debug {
    margin-top: 6px;
    padding: 4px 6px;
    background: var(--surface2);
    border-radius: 4px;
    font-size: 0.65em;
    font-family: 'Courier New', monospace;
    font-weight: 700;
    color: var(--text);
    min-height: 18px;
  }

  /* ===== ODOM ===== */
  .odom-big {
    font-size: 1.6em;
    font-weight: 700;
    color: #42a5f5;
  }
  .odom-unit { font-size: 0.5em; color: var(--muted); margin-left: 3px; }

  /* ===== RESPONSIVE ===== */
  @media (max-width: 960px) {
    .layout { grid-template-columns: 1fr; }
    .cam-container { min-height: 200px; }
    .flow-bar { overflow-x: auto; }
  }

  /* ===== COMPETITION FLOW ===== */
  .flow-section {
    max-width: 1400px;
    margin: 0 auto;
    padding: 0 14px 14px;
  }
  .flow-card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 18px;
    position: relative;
    overflow: hidden;
  }
  .flow-card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, #42a5f5, transparent);
    opacity: 0.5;
  }
  .flow-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 14px;
    font-weight: 700;
  }
  .flow-bar {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 6px;
    padding: 8px 0;
  }
  .flow-node {
    padding: 6px 10px;
    border-radius: 8px;
    font-size: 0.68em;
    font-weight: 600;
    letter-spacing: 0.2px;
    background: rgba(255,255,255,0.04);
    border: 1px solid rgba(255,255,255,0.06);
    color: #555;
    transition: all 0.4s cubic-bezier(0.4,0,0.2,1);
    cursor: default;
    white-space: nowrap;
    position: relative;
  }
  .flow-node.done {
    background: rgba(76,175,80,0.12);
    border-color: rgba(76,175,80,0.3);
    color: #81c784;
  }
  .flow-node.done::after {
    content: 'âœ“';
    position: absolute;
    top: -4px; right: -4px;
    width: 14px; height: 14px;
    background: #4caf50;
    border-radius: 50%;
    font-size: 9px;
    display: flex; align-items: center; justify-content: center;
    color: #fff;
  }
  .flow-node.active {
    background: linear-gradient(135deg, rgba(233,69,96,0.2), rgba(66,165,245,0.2));
    border-color: var(--accent);
    color: #fff;
    box-shadow: 0 0 20px rgba(233,69,96,0.25), 0 0 40px rgba(233,69,96,0.1);
    transform: scale(1.08);
    font-weight: 700;
  }
  @keyframes activeGlow {
    0%,100% { box-shadow: 0 0 15px rgba(233,69,96,0.2); }
    50% { box-shadow: 0 0 25px rgba(233,69,96,0.4), 0 0 50px rgba(233,69,96,0.15); }
  }
  .flow-node.active { animation: activeGlow 2s ease infinite; }
  .flow-arrow {
    color: #333;
    font-size: 0.6em;
    padding: 0 1px;
    transition: color 0.3s;
  }
  .flow-arrow.passed { color: #4caf50; }
  .flow-lap-label {
    flex-shrink: 0;
    padding: 4px 10px;
    border-radius: 12px;
    font-size: 0.55em;
    font-weight: 700;
    letter-spacing: 1px;
    text-transform: uppercase;
    margin: 0 6px;
  }
  .lap1-label { background: rgba(233,69,96,0.15); color: var(--accent); border: 1px solid rgba(233,69,96,0.2); }
  .lap2-label { background: rgba(66,165,245,0.15); color: #42a5f5; border: 1px solid rgba(66,165,245,0.2); }

  /* ===== CONTROLLER POPOUT ===== */
  .ctrl-popout-tab {
    position: fixed;
    right: 0;
    top: 50%;
    transform: translateY(-50%);
    background: #fff;
    padding: 10px 8px;
    border-radius: 10px 0 0 10px;
    border: 1px solid rgba(0,0,0,0.08);
    border-right: none;
    cursor: pointer;
    z-index: 200;
    writing-mode: vertical-rl;
    text-orientation: mixed;
    font-size: 0.7em;
    font-weight: 700;
    color: var(--accent);
    letter-spacing: 1px;
    transition: all 0.3s;
    box-shadow: -2px 0 8px rgba(0,0,0,0.06);
  }
  .ctrl-popout-tab:hover {
    background: rgba(30,102,245,0.05);
    padding-right: 12px;
  }
  .ctrl-drawer {
    position: fixed;
    right: -320px;
    top: 60px;
    bottom: 0;
    width: 300px;
    background: #fff;
    border-left: 1px solid rgba(0,0,0,0.08);
    z-index: 199;
    transition: right 0.4s cubic-bezier(0.4,0,0.2,1);
    overflow-y: auto;
    padding: 20px;
    box-shadow: -4px 0 20px rgba(0,0,0,0.08);
  }
  .ctrl-drawer.open { right: 0; }
  .ctrl-drawer h3 {
    font-size: 0.7em;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: var(--accent);
    margin-bottom: 14px;
    font-weight: 700;
  }
  .ctrl-map-row {
    display: flex;
    align-items: center;
    padding: 8px 0;
    border-bottom: 1px solid rgba(255,255,255,0.04);
  }
  .ctrl-map-row:last-child { border: none; }
  .ctrl-key {
    flex-shrink: 0;
    width: 60px;
    padding: 4px 8px;
    font-size: 0.75em;
    font-weight: 700;
    text-align: center;
    border-radius: 6px;
    background: var(--surface2);
    border: 1px solid rgba(0,0,0,0.1);
    color: #aaa;
    margin-right: 12px;
  }
  .ctrl-desc {
    font-size: 0.78em;
    color: #888;
  }
  .ctrl-section-label {
    font-size: 0.6em;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: #444;
    margin: 16px 0 8px;
    padding-bottom: 4px;
    border-bottom: 1px solid rgba(255,255,255,0.05);
  }

  /* ===== EVENT LOG ===== */
  .log-section {
    max-width: 1400px;
    margin: 0 auto;
    padding: 0 14px 14px;
  }
  .log-card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 14px 18px;
  }
  .log-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 10px;
    font-weight: 700;
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .event-log {
    font-family: 'JetBrains Mono', 'Consolas', monospace;
    font-size: 0.75em;
    height: 140px;
    overflow-y: auto;
    background: #e6e9ef;
    border-radius: 8px;
    padding: 12px;
    border: 1px solid rgba(0,0,0,0.05);
    color: #4c4f69;
  }
  .event-log::-webkit-scrollbar { width: 6px; }
  .event-log::-webkit-scrollbar-track { background: transparent; }
  .event-log::-webkit-scrollbar-thumb { background: rgba(0,0,0,0.1); border-radius: 3px; }
  .log-entry { margin-bottom: 4px; border-bottom: 1px solid rgba(0,0,0,0.02); padding-bottom: 4px; }
  .log-time { color: var(--accent); margin-right: 8px; font-weight: 600; }
  .log-val { font-weight: 700; color: #d20f39; }

  /* ===== PARAMETER DRAWER ===== */
  .param-popout-tab {
    position: fixed; left: 0; top: 50%; transform: translateY(-50%);
    background: #fff;
    padding: 10px 8px; border-radius: 0 10px 10px 0;
    border: 1px solid rgba(0,0,0,0.08); border-left: none;
    cursor: pointer; z-index: 200; writing-mode: vertical-lr;
    text-orientation: mixed; font-size: 0.7em; font-weight: 700;
    color: var(--accent); letter-spacing: 1px; transition: all 0.3s;
    box-shadow: 2px 0 8px rgba(0,0,0,0.06);
  }
  .param-popout-tab:hover { background: rgba(30,102,245,0.05); padding-left: 12px; }
  .param-drawer {
    position: fixed; left: -480px; top: 60px; bottom: 0; width: 420px;
    background: #fff;
    border-right: 1px solid rgba(0,0,0,0.08); z-index: 199;
    transition: left 0.4s cubic-bezier(0.4,0,0.2,1); overflow-y: auto;
    padding: 20px; box-shadow: 4px 0 20px rgba(0,0,0,0.08);
  }
  .param-drawer.open { left: 0; }
  .param-drawer h3 {
    font-size: 0.85em; text-transform: uppercase; letter-spacing: 2px;
    color: var(--accent); margin-bottom: 8px; font-weight: 700;
  }
  .param-drawer .note { font-size: 0.8em; color: #666; margin-bottom: 12px; }
  .param-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: #666;
    margin-bottom: 12px;
    font-weight: 700;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .param-toolbar {
    display: flex; gap: 8px; align-items: center;
    margin-bottom: 12px;
  }
  .param-refresh-btn, .param-load-btn {
    padding: 5px 14px;
    border-radius: 6px;
    border: 1px solid rgba(30,102,245,0.2);
    background: rgba(30,102,245,0.05);
    color: var(--accent);
    cursor: pointer;
    font-size: 0.75em;
    font-weight: 700;
    transition: all 0.2s;
    font-family: inherit;
  }
  .param-refresh-btn:hover, .param-load-btn:hover {
    background: rgba(30,102,245,0.1); border-color: var(--accent);
  }
  .param-name[title] {
    position: relative;
    cursor: help;
    border-bottom: 1px dotted rgba(0,0,0,0.25);
  }
  .param-name[title]:hover::after {
    content: attr(title);
    position: fixed;
    transform: translate(0, 24px);
    background: #333;
    border: 1px solid rgba(0,0,0,0.3);
    color: #fff;
    padding: 6px 10px;
    border-radius: 6px;
    font-size: 0.85em;
    font-weight: 500;
    max-width: 300px;
    white-space: normal;
    z-index: 99999;
    pointer-events: none;
    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    animation: tipFade 0.15s ease;
  }
  @keyframes tipFade { from { opacity: 0; transform: translateY(-2px); } to { opacity: 1; transform: translateY(0); } }
  .param-node-block {
    margin-bottom: 8px;
    border: 1px solid rgba(0,0,0,0.08);
    border-radius: 10px;
    overflow: hidden;
  }
  .param-node-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 14px;
    background: #f8f9fb;
    cursor: pointer;
    transition: background 0.2s;
  }
  .param-node-header:hover { background: #f0f1f5; }
  .param-node-name {
    font-size: 0.8em;
    font-weight: 700;
    color: var(--accent);
  }
  .param-node-count {
    font-size: 0.65em;
    color: #888;
    font-weight: 600;
  }
  .param-node-body {
    max-height: 0;
    overflow: hidden;
    transition: max-height 0.35s ease;
  }
  .param-node-body.open {
    max-height: 2000px;
  }
  .param-row {
    display: grid;
    grid-template-columns: 1fr 80px 38px 38px;
    align-items: center;
    padding: 6px 14px;
    border-top: 1px solid rgba(0,0,0,0.04);
    gap: 6px;
    position: relative;
  }
  .param-name {
    font-size: 0.78em;
    color: #333;
    font-weight: 600;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .param-val {
    width: 100%;
    box-sizing: border-box;
    padding: 5px 8px;
    border-radius: 4px;
    border: 1px solid rgba(0,0,0,0.12);
    background: #f8f9fb;
    color: #333;
    font-size: 0.8em;
    font-family: 'Segoe UI', Roboto, sans-serif;
    outline: none;
    transition: border-color 0.2s;
  }
  .param-val:focus { border-color: var(--accent); box-shadow: 0 0 0 2px rgba(30,102,245,0.15); }
  .param-set-btn {
    padding: 4px 0;
    border-radius: 4px;
    border: 1px solid rgba(64,160,43,0.3);
    background: rgba(64,160,43,0.08);
    color: var(--success);
    cursor: pointer;
    font-size: 0.65em;
    font-weight: 700;
    transition: all 0.2s;
    font-family: inherit;
    text-align: center;
  }
  .param-set-btn:hover { background: rgba(64,160,43,0.15); border-color: var(--success); }
  .param-get-btn {
    padding: 4px 0;
    border-radius: 4px;
    border: 1px solid rgba(30,102,245,0.3);
    background: rgba(30,102,245,0.08);
    color: var(--accent);
    cursor: pointer;
    font-size: 0.65em;
    font-weight: 700;
    transition: all 0.2s;
    font-family: inherit;
    text-align: center;
  }
  .param-get-btn:hover { background: rgba(30,102,245,0.15); border-color: var(--accent); }
  .param-status {
    position: absolute;
    right: 14px;
    bottom: -8px;
    font-size: 0.65em;
    padding: 2px 6px;
    border-radius: 3px;
    animation: logFade 0.3s ease;
  }
  .param-status.ok { color: var(--success); }
  .param-status.err { color: var(--danger); }
  .param-empty {
    text-align: center;
    padding: 20px;
    color: #888;
    font-size: 0.8em;
  }
  .param-save-defaults-btn {
    display: block;
    width: 100%;
    padding: 10px 14px;
    border-radius: 8px;
    border: 1px solid rgba(223, 142, 29, 0.3);
    background: rgba(223, 142, 29, 0.08);
    color: var(--warning);
    cursor: pointer;
    font-size: 0.82em;
    font-weight: 700;
    transition: all 0.2s;
    font-family: inherit;
    margin-top: 14px;
    letter-spacing: 0.3px;
  }
  .param-save-defaults-btn:hover {
    background: rgba(223, 142, 29, 0.18);
    border-color: var(--warning);
    box-shadow: 0 0 12px rgba(223, 142, 29, 0.15);
  }
  .param-save-defaults-btn:active {
    transform: scale(0.98);
  }
  .param-save-defaults-btn.saving {
    opacity: 0.6;
    pointer-events: none;
  }
  .param-save-defaults-btn.success {
    background: rgba(64, 160, 43, 0.15);
    border-color: var(--success);
    color: var(--success);
  }
  .param-save-defaults-btn.error {
    background: rgba(210, 15, 57, 0.1);
    border-color: var(--danger);
    color: var(--danger);
  }
</style>
</head>
<body>

<!-- BACKGROUND ORBS -->
<div class="bg-orbs">
  <div class="orb orb-1"></div>
  <div class="orb orb-2"></div>
  <div class="orb orb-3"></div>
</div>

<!-- WARNING OVERLAY -->
<div id="warning-overlay" class="warning-overlay"></div>

<!-- HEADER -->
<div class="header">
  <h1>🤖 RISA-Bot <span style="font-size:0.6em;color:rgba(255,255,255,0.4);vertical-align:middle;">v2.0</span></h1>
  <div class="conn-badge">
    <span class="header-meta" id="uptimeText">00:00:00</span>
    <span class="latency-badge" id="latencyText">— ms</span>
    <span class="conn-dot" id="connDot"></span>
    <span id="connText">Connecting...</span>
  </div>
</div>

<!-- MAIN LAYOUT -->
<div class="layout">

  <!-- ===== LEFT COLUMN ===== -->
  <div class="col">

    <!-- State Machine -->
    <div class="card">
      <h3 style="display:flex;align-items:center;justify-content:space-between;">State Machine <span class="mode-badge" id="modeBadge">MANUAL</span></h3>
      <div style="margin-top:4px;">
        <span class="state-badge" id="stateBadge">—</span>
      </div>
      <div style="margin-top:8px;">
        <span class="lap-badge" id="lapBadge">Lap —</span>
      </div>
      <div class="info-row">
        <span>⏱️ <span id="stateTime">0</span>s</span>
        <span>📝 <span id="stateDist">0.00</span>m</span>
        <span>⏱️ <span id="lapTimer" style="font-variant-numeric:tabular-nums;">00:00</span></span>
      </div>
      <div id="stopReasonRow" style="margin-top:6px;">
        <span id="stopBadge" style="display:inline-block;padding:3px 10px;border-radius:4px;font-size:0.75em;font-weight:700;letter-spacing:0.5px;background:rgba(166,227,161,0.15);color:#a6e3a1;">DRIVING</span>
      </div>

      <!-- Reset & Lap Override Buttons -->
      <div style="margin-top:14px; display:flex; gap:6px;">
        <button class="state-action-btn reset" onclick="sendCompCmd('RESET')" style="flex:1; padding:6px 4px; border-radius:6px; border:1px solid rgba(223,142,29,0.3); font-size:0.7em; font-weight:700; cursor:pointer; background:rgba(223,142,29,0.08); color:var(--warning); font-family:inherit; text-transform:uppercase; letter-spacing:0.5px; transition:all 0.2s;">
          Reset
        </button>
        <button class="state-action-btn lap1" onclick="sendCompCmd('LAP1')" style="flex:1; padding:6px 4px; border-radius:6px; border:1px solid rgba(30,102,245,0.3); font-size:0.7em; font-weight:700; cursor:pointer; background:rgba(30,102,245,0.08); color:var(--accent); font-family:inherit; text-transform:uppercase; letter-spacing:0.5px; transition:all 0.2s;">
          Lap 1
        </button>
        <button class="state-action-btn lap2" onclick="sendCompCmd('LAP2')" style="flex:1; padding:6px 4px; border-radius:6px; border:1px solid rgba(66,165,245,0.3); font-size:0.7em; font-weight:700; cursor:pointer; background:rgba(66,165,245,0.08); color:#42a5f5; font-family:inherit; text-transform:uppercase; letter-spacing:0.5px; transition:all 0.2s;">
          Lap 2
        </button>
      </div>
    </div>

    <!-- Traffic Light -->
    <div class="card">
      <h3>Traffic Light</h3>
      <div class="traffic-light">
        <div class="tl-circle tl-red" id="tlRed"></div>
        <div class="tl-circle tl-yellow" id="tlYellow"></div>
        <div class="tl-circle tl-green" id="tlGreen"></div>
      </div>
      <div class="tl-label" id="tlText">unknown</div>
    </div>

    <!-- IMU / Attitude -->
    <div class="card" id="imuCard">
      <h3 style="display:flex;align-items:center;justify-content:space-between;">
        🧭 IMU Attitude
        <button onclick="calibrateIMU()" id="imuCalBtn"
          style="padding:4px 10px;border-radius:6px;border:1px solid rgba(30,102,245,0.3);font-size:0.7em;font-weight:700;cursor:pointer;background:rgba(30,102,245,0.08);color:var(--accent);font-family:inherit;transition:all 0.2s;">
          ⚙ Calibrate
        </button>
      </h3>

      <!-- Roll -->
      <div class="s-row">
        <span class="s-label">Roll</span>
        <span class="s-val" id="imuRollVal" style="font-variant-numeric:tabular-nums;">0.00°</span>
      </div>
      <div class="meter" style="margin-bottom:8px;">
        <div class="meter-fill" id="imuRollBar" style="width:50%;background:linear-gradient(90deg,#1565c0,#42a5f5);"></div>
      </div>

      <!-- Pitch -->
      <div class="s-row">
        <span class="s-label">Pitch</span>
        <span class="s-val" id="imuPitchVal" style="font-variant-numeric:tabular-nums;">0.00°</span>
      </div>
      <div class="meter" style="margin-bottom:8px;">
        <div class="meter-fill" id="imuPitchBar" style="width:50%;background:linear-gradient(90deg,#e65100,#ff9800);"></div>
      </div>

      <!-- Yaw -->
      <div class="s-row">
        <span class="s-label">Yaw</span>
        <span class="s-val" id="imuYawVal" style="font-variant-numeric:tabular-nums;">0.00°</span>
      </div>
      <div class="meter">
        <div class="meter-fill" id="imuYawBar" style="width:50%;background:linear-gradient(90deg,#4a148c,#ce93d8);"></div>
      </div>

      <div id="imuCalMsg" style="margin-top:8px;font-size:0.7em;color:var(--muted);min-height:16px;"></div>
    </div>

    <!-- Manual Control -->
    <div class="card">
      <h3>Manual Control</h3>
      <div class="speed-display">
        <div class="num" id="speedPct">25%</div>
        <div class="unit">Drive Speed</div>
        <div class="gear-dots">
          <div class="gear-dot active" id="gear0"></div>
          <div class="gear-dot" id="gear1"></div>
          <div class="gear-dot" id="gear2"></div>
          <div class="gear-dot" id="gear3"></div>
        </div>
      </div>
      <div class="meter"><div class="meter-fill meter-orange" id="speedBar" style="width:25%"></div></div>
      <div style="margin-top:6px;font-size:0.65em;color:#444;text-align:center;">D-pad ▲/▼ to shift</div>
      <div style="margin-top:12px;">
        <div class="s-row">
          <span class="s-label">Selector</span>
          <span class="ctrl-state-badge" id="ctrlState">LANE_FOLLOW</span>
        </div>
        <div style="font-size:0.65em;color:#444;">LB / RB to cycle</div>
      </div>
    </div>

    <!-- Record & Playback -->
    <div class="card">
      <h3 style="display:flex;align-items:center;justify-content:space-between;">🎬 Record &amp; Playback <span class="rp-state-badge idle" id="rpStateBadge">IDLE</span></h3>
      
      <!-- Control Buttons -->
      <div class="rp-buttons" style="display:flex; gap:8px; margin-top:10px;">
        <button class="rp-btn record" id="rpBtnRecord" onclick="rpCmd('record')" style="flex:1; padding:8px 4px; border-radius:8px; border:1px solid rgba(255,82,82,0.3); font-size:0.8em; font-weight:700; cursor:pointer; background:rgba(255,82,82,0.1); color:#d20f39; display:flex; flex-direction:column; align-items:center; gap:4px; font-family:inherit;">
          <span class="icon">🔴</span>
          <span class="label" style="font-size:0.7em; letter-spacing:0.5px; text-transform:uppercase;">Record</span>
        </button>
        <button class="rp-btn stop" id="rpBtnStop" onclick="rpCmd('stop')" style="flex:1; padding:8px 4px; border-radius:8px; border:1px solid rgba(255,215,64,0.3); font-size:0.8em; font-weight:700; cursor:pointer; background:rgba(255,215,64,0.1); color:#df8e1d; display:flex; flex-direction:column; align-items:center; gap:4px; font-family:inherit;">
          <span class="icon">⏹</span>
          <span class="label" style="font-size:0.7em; letter-spacing:0.5px; text-transform:uppercase;">Stop</span>
        </button>
        <button class="rp-btn play" id="rpBtnPlay" onclick="rpCmd('playback')" style="flex:1; padding:8px 4px; border-radius:8px; border:1px solid rgba(105,240,174,0.3); font-size:0.8em; font-weight:700; cursor:pointer; background:rgba(105,240,174,0.1); color:#40a02b; display:flex; flex-direction:column; align-items:center; gap:4px; font-family:inherit;">
          <span class="icon">▶</span>
          <span class="label" style="font-size:0.7em; letter-spacing:0.5px; text-transform:uppercase;">Play</span>
        </button>
        <button class="rp-btn save" id="rpBtnSave" onclick="rpCmd('save')" style="flex:1; padding:8px 4px; border-radius:8px; border:1px solid rgba(30,144,255,0.3); font-size:0.8em; font-weight:700; cursor:pointer; background:rgba(30,144,255,0.1); color:#1e90ff; display:flex; flex-direction:column; align-items:center; gap:4px; font-family:inherit;">
          <span class="icon">💾</span>
          <span class="label" style="font-size:0.7em; letter-spacing:0.5px; text-transform:uppercase;">Save</span>
        </button>
      </div>

      <!-- Progress Bar (visible during playback) -->
      <div class="progress-track" id="rpProgress" style="display:none; height:6px; background:var(--surface2); border-radius:3px; overflow:hidden; margin-top:10px; margin-bottom:10px;">
        <div class="progress-fill" id="rpProgressFill" style="width:0%; height:100%; border-radius:3px; background:linear-gradient(90deg, var(--accent), #40a02b); transition:width 0.15s ease;"></div>
      </div>

      <!-- Buffer / Progress Info -->
      <div class="rp-info" style="display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:10px;">
        <div class="rp-info-item" style="background:var(--surface2); border-radius:8px; padding:8px; text-align:center;">
          <div class="value" id="rpBufferSize" style="font-family:'JetBrains Mono',monospace; font-size:1.4em; font-weight:800; color:var(--accent); line-height:1.2;">0</div>
          <div class="label" style="font-size:0.65em; color:#666; text-transform:uppercase; letter-spacing:1px; margin-top:2px;">Samples</div>
        </div>
        <div class="rp-info-item" style="background:var(--surface2); border-radius:8px; padding:8px; text-align:center;">
          <div class="value" id="rpDuration" style="font-family:'JetBrains Mono',monospace; font-size:1.4em; font-weight:800; color:var(--accent); line-height:1.2;">0.0</div>
          <div class="label" style="font-size:0.65em; color:#666; text-transform:uppercase; letter-spacing:1px; margin-top:2px;">Duration (s)</div>
        </div>
      </div>
    </div>

    <!-- Lane Following -->
    <div class="card">
      <h3>Lane Following</h3>
      <div class="s-row">
        <span class="s-label">Error</span>
        <span class="s-val" id="laneErr">0.000</span>
      </div>
      <div class="meter"><div class="meter-fill meter-blue" id="laneBar" style="width:50%"></div></div>
      <div class="s-row" style="margin-top:6px;">
        <span class="s-label">Linear X</span>
        <span class="s-val" id="cmdLinX">0.000</span>
      </div>
      <div class="s-row">
        <span class="s-label">Angular Z</span>
        <span class="s-val" id="cmdAngZ">0.000</span>
      </div>
    </div>

  </div>

  <!-- ===== CENTER COLUMN â€” CAMERA ===== -->
  <div class="col">
    <div class="card cam-card">
      <h3>Camera Feed</h3>
      <button class="cam-toggle" id="camBtn" onclick="toggleCam()">📷 Enable Camera</button>
      <div class="cam-tabs" id="camTabs" style="display:none;">
        <div class="cam-tab active" onclick="setCamView('raw', this)">Raw</div>
        <div class="cam-tab" onclick="setCamView('line_follower', this)">Lane Lines</div>
        <div class="cam-tab" onclick="setCamView('traffic_light', this)">Traffic Light</div>
        <div class="cam-tab" onclick="setCamView('obstacle', this)">Obstacle</div>
        <div class="cam-tab" onclick="setCamView('signage', this)">Signage</div>
      </div>
      <div class="cam-container" id="camContainer">
        <div class="cam-off" id="camOff">
          <div class="icon">📷</div>
          Click above to enable
        </div>
        <img id="camImg" src="" alt="Camera" style="display:none;">
      </div>
    </div>

    <!-- LiDAR 2D Top-Down View -->
    <div class="card">
      <div style="display:flex; justify-content:space-between; align-items:center;">
        <h3 style="margin:0;">LiDAR Top View</h3>
        <span id="lidarStatus" style="font-size:0.8em; font-weight:600; color:#888;">● Waiting</span>
      </div>
      <canvas id="lidarCanvas" width="320" height="320" style="width:100%; margin-top:8px; border-radius:12px; background:#0a0a0f; border:1px solid rgba(0,0,0,0.1);"></canvas>
    </div>
  </div>

  <!-- ===== RIGHT COLUMN ===== -->
  <div class="col">

    <!-- Sensors -->
    <div class="card">
      <h3>Sensors</h3>
      <div class="s-row"><span class="s-label">LiDAR</span><span class="s-val"><span class="dot dot-gray" id="dotLidar"></span><span id="valLidar">—</span></span></div>
      <div class="s-row"><span class="s-label">Camera</span><span class="s-val"><span class="dot dot-gray" id="dotCam"></span><span id="valCam">—</span></span></div>
      <div class="s-row"><span class="s-label">Fused</span><span class="s-val"><span class="dot dot-gray" id="dotFused"></span><span id="valFused">—</span></span></div>
      <div class="s-row"><span class="s-label">Boom Gate</span><span class="s-val"><span class="dot dot-gray" id="dotGate"></span><span id="valGate">—</span></span></div>
      <div class="s-row"><span class="s-label">Tunnel</span><span class="s-val"><span class="dot dot-gray" id="dotTunnel"></span><span id="valTunnel">—</span></span></div>
      <div class="s-row"><span class="s-label">Obstruction</span><span class="s-val"><span class="dot dot-gray" id="dotObst"></span><span id="valObst">—</span></span></div>
      <div class="s-row"><span class="s-label">Parking</span><span class="s-val"><span class="dot dot-gray" id="dotPark"></span><span id="valPark">—</span></span></div>
      <div class="s-row"><span class="s-label">Signage</span><span class="s-val"><span class="dot dot-gray" id="dotSignage"></span><span id="valSignage">—</span></span></div>
      <div class="s-row"><span class="s-label">Health</span><span class="s-val"><span class="dot dot-gray" id="dotHealth"></span><span id="valHealth">—</span></span></div>
      <div class="s-row"><span class="s-label">Stale Streams</span><span class="s-val" id="valStale">—</span></div>
    </div>

    <!-- Odometry -->
    <div class="card">
      <h3>Odometry</h3>
      <div class="s-row"><span class="s-label">Total Dist</span><span class="s-val" style="font-weight:bold;"><span id="odomDist">0.00</span> m</span></div>
      <div class="s-row"><span class="s-label">Speed</span><span class="s-val" id="odomSpeed">0.000 m/s</span></div>
      <div class="s-row"><span class="s-label">Pos X</span><span class="s-val" id="odomX">0.00 m</span></div>
      <div class="s-row"><span class="s-label">Pos Y</span><span class="s-val" id="odomY">0.00 m</span></div>
      <div class="s-row"><span class="s-label">Heading</span><span class="s-val" id="odomYaw">0°</span></div>
      <button onclick="fetch('/api/reset_odom',{method:'POST'}).then(update)" style="margin-top:10px;width:100%;padding:6px;background:#333;color:#fff;border:1px solid #555;border-radius:4px;cursor:pointer;">Reset Odometry</button>
    </div>

    <!-- Controller -->
    <div class="card">
      <h3>Controller</h3>
      <div class="ctrl-grid">
        <div class="ctrl-btn" id="btnLB">LB</div>
        <div class="ctrl-btn" id="btnUp">▲</div>
        <div class="ctrl-btn" id="btnRB">RB</div>
        <div class="ctrl-btn" id="btnLeft">◀</div>
        <div class="ctrl-btn" id="btnStart">STA</div>
        <div class="ctrl-btn" id="btnRight">▶</div>
        <div class="ctrl-btn" id="btnX">X</div>
        <div class="ctrl-btn" id="btnDown">▼</div>
        <div class="ctrl-btn" id="btnY">Y</div>
        <div class="ctrl-btn" id="btnA">A</div>
        <div class="ctrl-btn" id="btnLT">LT</div>
        <div class="ctrl-btn" id="btnB">B</div>
        <div class="ctrl-btn" id="btnRT" style="grid-column:3;">RT</div>
      </div>
      <div class="joy-info">
        <div class="joy-container">
          <div class="joy-circle"><div class="joy-dot" id="joyDotL"></div></div>
          <div class="joy-label" id="joyValL">L: 0.0, 0.0</div>
        </div>
        <div class="joy-container">
          <div class="joy-circle"><div class="joy-dot" id="joyDotR"></div></div>
          <div class="joy-label" id="joyValR">R: 0.0, 0.0</div>
        </div>
      </div>
      <div class="btn-debug" id="btnDebug">Press a button to see index…</div>
    </div>

  </div>
</div>

<!-- ===== BEHAVIOR PRIORITY VISUALIZER ===== -->
<div class="flow-section">
  <div class="flow-card">
    <h3>🧠 Hybrid Action Priority <span style="font-size:0.85em;color:#444;font-weight:400;text-transform:none;letter-spacing:0;"> — Highlighted block is the currently active overriding behavior</span></h3>
    <div class="flow-bar" id="flowBar">
      <div class="flow-node" id="flow_LANE_FOLLOW">Lane Follow</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_TUNNEL">Tunnel</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_OBSTRUCTION">Obstruction</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_BOOM_GATE">Boom Gate</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_TRAFFIC_LIGHT">Traffic Light</div>
      <span class="flow-arrow">◀</span>
      <div class="flow-node" id="flow_MANUAL">Manual Control</div>
    </div>
  </div>
</div>

<!-- ===== EVENT LOG ===== -->
<div class="log-section">
  <div class="log-card">
    <h3><span style="opacity:0.6">📝 </span> Event Log</h3>
    <div class="log-scroll" id="logScroll">
      <div class="log-entry"><span class="log-time">--:--:--</span><span class="log-event">Dashboard started</span></div>
    </div>
  </div>
</div>

<!-- ===== PARAMETER TUNING ===== -->
<!-- ===== PARAMETER TUNING DRAWER ===== -->
<div class="param-popout-tab" onclick="toggleParamDrawer()">⚙️ Parameters</div>
<div class="param-drawer" id="paramDrawer">
  <h3>⚙️ Parameter Tuning</h3>
  <div class="note">💡 Changes apply instantly to nodes but revert to defaults upon restart.</div>
  <div id="paramContainer"></div>
  <button class="param-save-defaults-btn" id="saveDefaultsBtn" onclick="saveDefaults()">💾 Save Current as Default</button>
  <div id="saveDefaultsStatus" style="text-align:center;font-size:0.78em;margin-top:6px;min-height:20px;"></div>
</div>

<!-- ===== CONTROLLER POPOUT ===== -->
<div class="ctrl-popout-tab" onclick="toggleCtrlDrawer()">🎮 Controls</div>
<div class="ctrl-drawer" id="ctrlDrawer">
  <h3>🎮 Control Mapping</h3>
  <div class="ctrl-section-label">Driving</div>
  <div class="ctrl-map-row"><span class="ctrl-key">L Stick Y</span><span class="ctrl-desc">Throttle (forward/reverse)</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">R Stick X</span><span class="ctrl-desc">Steering (left/right)</span></div>
  <div class="ctrl-section-label">Speed</div>
  <div class="ctrl-map-row"><span class="ctrl-key">D-Pad ▲</span><span class="ctrl-desc">Speed up (shift gear)</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">D-Pad ▼</span><span class="ctrl-desc">Speed down (shift gear)</span></div>
  <div class="ctrl-section-label">Mode</div>
  <div class="ctrl-map-row"><span class="ctrl-key">Y</span><span class="ctrl-desc">Toggle Auto/Manual mode</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">Start</span><span class="ctrl-desc">Toggle Auto/Manual mode</span></div>
  <div class="ctrl-section-label">Challenge</div>
  <div class="ctrl-map-row"><span class="ctrl-key">RB</span><span class="ctrl-desc">Next challenge state</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">LB</span><span class="ctrl-desc">Previous challenge state</span></div>
  <div class="ctrl-section-label">Record & Playback</div>
  <div class="ctrl-map-row"><span class="ctrl-key">A</span><span class="ctrl-desc">Record / Stop Recording</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">X</span><span class="ctrl-desc">Play / Stop Playback</span></div>
  <div class="ctrl-section-label">Camera</div>
  <div class="ctrl-map-row"><span class="ctrl-key">R Stick Y</span><span class="ctrl-desc">Camera tilt (if enabled)</span></div>
</div>

<script>
let eventLog = [];
const PRIORITY_ORDER = ['LANE_FOLLOW', 'TUNNEL', 'OBSTRUCTION', 'BOOM_GATE', 'TRAFFIC_LIGHT', 'MANUAL'];
let lastState = '';
let lastLap = 0;
let lapStartTime = Date.now();
const dashStartTime = Date.now();
let lastStopReason = '';

// Session uptime timer
setInterval(() => {
  const elapsed = Math.floor((Date.now() - dashStartTime) / 1000);
  const h = String(Math.floor(elapsed / 3600)).padStart(2, '0');
  const m = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
  const s = String(elapsed % 60).padStart(2, '0');
  document.getElementById('uptimeText').textContent = h + ':' + m + ':' + s;
  // Lap timer
  const lapElapsed = Math.floor((Date.now() - lapStartTime) / 1000);
  const lm = String(Math.floor(lapElapsed / 60)).padStart(2, '0');
  const ls = String(lapElapsed % 60).padStart(2, '0');
  document.getElementById('lapTimer').textContent = lm + ':' + ls;
}, 1000);

function toggleCtrlDrawer() {
  document.getElementById('ctrlDrawer').classList.toggle('open');
}

function toggleParamDrawer() {
  document.getElementById('paramDrawer').classList.toggle('open');
  document.querySelector('.layout').classList.toggle('drawer-open');
}

function addLogEntry(text) {
  const now = new Date();
  const ts = now.toLocaleTimeString('en-GB');
  eventLog.unshift({time: ts, text: text});
  if (eventLog.length > 30) eventLog.pop();
  const container = document.getElementById('logScroll');
  container.innerHTML = eventLog.map(e =>
    `<div class="log-entry"><span class="log-time">${e.time}</span><span class="log-event">${e.text}</span></div>`
  ).join('');
}

function updateFlow(currentState) {
  const activeIdx = PRIORITY_ORDER.indexOf(currentState);
  
  PRIORITY_ORDER.forEach((s, i) => {
    const el = document.getElementById('flow_' + s);
    if (!el) return;
    
    el.classList.remove('active', 'done');
    // If it's the active state, highlight it
    if (i === activeIdx) {
      el.classList.add('active');
    } 
    // If it's a lower priority state than current, mark it as "done" (overridden)
    else if (i < activeIdx) {
      el.classList.add('done');
    }
  });

  // Color arrows. Arrow i is between node i and i+1.
  const arrows = document.querySelectorAll('.flow-arrow');
  arrows.forEach((a, i) => {
    // If the active state is higher than i, the arrow is lit up implying flow of priority
    a.classList.toggle('passed', i < activeIdx);
  });
}

function toggleCam() {
  const b = document.getElementById('camBtn');
  const d = document.getElementById('camContainer');
  const t = document.getElementById('camTabs');
  const off = document.getElementById('camOff');
  const img = document.getElementById('camImg');
  if(b.classList.contains('active')) {
    b.classList.remove('active');
    b.textContent = String.fromCodePoint(0x1F4F7) + ' Enable Camera';
    d.classList.remove('active');
    t.style.display = 'none';
    off.style.display = 'flex';
    img.style.display = 'none';
    img.src = '';
  } else {
    b.classList.add('active');
    b.textContent = String.fromCodePoint(0x1F4F7) + ' Disable Camera';
    d.classList.add('active');
    t.style.display = 'flex';
    off.style.display = 'none';
    img.style.display = 'block';
    img.src = '/camera_feed?' + new Date().getTime();
    // Trigger auto_toggle_debug on initial enable (default view is 'raw')
    fetch('/api/set_cam_view?view=raw');
  }
}

function setCamView(view, btn) {
  document.querySelectorAll('.cam-tab').forEach(t => t.classList.remove('active'));
  btn.classList.add('active');
  fetch('/api/set_cam_view?view=' + encodeURIComponent(view)).then(() => {
    // Reload MJPEG stream to pick up the new view immediately
    const img = document.getElementById('camImg');
    if (img && img.style.display !== 'none') {
      img.src = '/camera_feed?' + Date.now();
    }
  });
}

function rpCmd(action) {
  fetch('/api/record_playback', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: action})
  }).catch(() => {});
}

let imuCalStep = 0;
function calibrateIMU() {
  const btn = document.getElementById('imuCalBtn');
  const msg = document.getElementById('imuCalMsg');
  
  if (imuCalStep === 0) {
    // Step 1: Zero
    btn.textContent = 'Zero Axes';
    btn.style.background = 'rgba(223,142,29,0.15)';
    btn.style.color = 'var(--warning)';
    msg.innerHTML = '<b>Step 1:</b> Place robot perfectly flat on the ground and click <b>Zero Axes</b>.';
    msg.style.color = 'var(--text)';
    imuCalStep = 1;
    
  } else if (imuCalStep === 1) {
    // Execute Zero -> prompt Pitch
    btn.disabled = true;
    fetch('/api/calibrate_imu', { method: 'POST', body: JSON.stringify({action: 'zero'}) })
      .then(r => r.json())
      .then(data => {
        btn.disabled = false;
        btn.textContent = 'Set Pitch +90°';
        btn.style.background = 'rgba(30,102,245,0.15)';
        btn.style.color = 'var(--accent)';
        msg.innerHTML = '✅ Axes Zeroed.<br><b>Step 2:</b> Point robot nose straight UP (90°) and click <b>Set Pitch +90°</b>.';
        imuCalStep = 2;
      }).catch(e => { btn.disabled = false; msg.innerHTML = '❌ Error'; imuCalStep = 0; });
      
  } else if (imuCalStep === 2) {
    // Execute Pitch -> prompt Roll
    btn.disabled = true;
    fetch('/api/calibrate_imu', { method: 'POST', body: JSON.stringify({action: 'set_scale', axis: 'pitch', target: 90.0}) })
      .then(r => r.json())
      .then(data => {
        btn.disabled = false;
        btn.textContent = 'Set Roll +90°';
        msg.innerHTML = '✅ Pitch Mapped.<br><b>Step 3:</b> Tilt robot 90° onto its RIGHT side and click <b>Set Roll +90°</b>.';
        imuCalStep = 3;
      }).catch(e => { btn.disabled = false; msg.innerHTML = '❌ Error'; imuCalStep = 0; });
      
  } else if (imuCalStep === 3) {
    // Execute Roll -> Finish
    btn.disabled = true;
    fetch('/api/calibrate_imu', { method: 'POST', body: JSON.stringify({action: 'set_scale', axis: 'roll', target: 90.0}) })
      .then(r => r.json())
      .then(data => {
        btn.disabled = false;
        btn.textContent = '⚙ Calibrate Wizard';
        btn.style.background = 'rgba(30,102,245,0.08)';
        msg.innerHTML = '✅ Calibration Complete!<br><span style="color:var(--warning)">Remember to open the Right Drawer and click <b>Save Defaults</b> to persist.</span>';
        imuCalStep = 0;
      }).catch(e => { btn.disabled = false; msg.innerHTML = '❌ Error'; imuCalStep = 0; });
  }
}

function sendCompCmd(cmd) {
  fetch('/api/reset_competition', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({command: cmd})
  })
  .then(r => r.json())
  .then(data => {
    if(data.ok) {
      addLogEntry(`Sent command: <span class="log-val">${cmd}</span>`);
      update();
    } else {
      addLogEntry(`⚠️ Error: ${data.error}`);
    }
  })
  .catch(err => {
    addLogEntry(`⚠️ Fetch Error: ${err}`);
  });
}

let last_data_time = 0;
function update() {
  const fetchStart = performance.now();
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      const latency = Math.round(performance.now() - fetchStart);
      document.getElementById('latencyText').textContent = latency + ' ms';
      document.getElementById('connDot').style.background = '#4caf50';
      document.getElementById('connText').textContent = 'Connected';

      // State
      const sb = document.getElementById('stateBadge');
      const currentState = d.state;
      sb.textContent = currentState;
      sb.className = 'state-badge state-' + currentState;

      // Update competition flow
      updateFlow(currentState);

      // Log state changes
      if (currentState !== lastState && lastState !== '') {
        addLogEntry(`State: <span class="log-val">${lastState}</span> ➔ <span class="log-val">${currentState}</span>`);
      }
      lastState = currentState;

      const mb = document.getElementById('modeBadge');
      const newMode = d.auto_mode ? 'AUTO' : 'MANUAL';
      const oldMode = mb.textContent;
      mb.textContent = newMode;
      mb.className = 'mode-badge mode-' + newMode;
      if (oldMode && oldMode !== newMode) {
        addLogEntry(`Mode: <span class="log-val">${newMode}</span>`);
      }

      document.getElementById('lapBadge').textContent = 'Lap ' + d.lap;
      // Lap timer reset
      if (d.lap !== lastLap && lastLap !== 0) {
        lapStartTime = Date.now();
        addLogEntry(`Lap <span class="log-val">${d.lap}</span> started`);
      }
      lastLap = d.lap;
      document.getElementById('stateTime').textContent = d.state_time;
      document.getElementById('stateDist').textContent = d.state_dist;

      // Stop reason badge
      const stopEl = document.getElementById('stopBadge');
      const sr = d.stop_reason || '';
      if (sr.length > 0) {
        stopEl.textContent = '⛔ ' + sr;
        stopEl.style.background = 'rgba(243,139,168,0.2)';
        stopEl.style.color = '#f38ba8';
        if (sr !== lastStopReason && lastStopReason === '') {
          addLogEntry(`⛔ Stopped: <span class="log-val">${sr}</span>`);
        }
      } else {
        stopEl.textContent = 'DRIVING';
        stopEl.style.background = 'rgba(64,160,43,0.15)';
        stopEl.style.color = '#40a02b';
        if (lastStopReason && lastStopReason.length > 0) {
          addLogEntry(`✅ Resumed driving`);
        }
      }
      lastStopReason = sr;

      // Traffic light
      ['Red','Yellow','Green'].forEach(c => {
        document.getElementById('tl'+c).classList.toggle('active', d.traffic_light === c.toLowerCase());
      });
      document.getElementById('tlText').textContent = d.traffic_light;

      // IMU Attitude (roll/pitch/yaw)
      (function() {
        const roll  = d.imu_roll  || 0;
        const pitch = d.imu_pitch || 0;
        const yaw   = d.imu_yaw   || 0;
        document.getElementById('imuRollVal').textContent  = roll.toFixed(2)  + '°';
        document.getElementById('imuPitchVal').textContent = pitch.toFixed(2) + '°';
        document.getElementById('imuYawVal').textContent   = yaw.toFixed(2)   + '°';
        // Map [-90,90] → [0%,100%] for bars (centre = 50%)
        const toBar = v => Math.min(100, Math.max(0, 50 + (v / 90) * 50));
        document.getElementById('imuRollBar').style.width  = toBar(roll)  + '%';
        document.getElementById('imuPitchBar').style.width = toBar(pitch) + '%';
        // Yaw maps [-180,180] → [0%,100%]
        document.getElementById('imuYawBar').style.width   = Math.min(100, Math.max(0, 50 + (yaw / 180) * 50)) + '%';
        // Colour pitch bar red when steep (>10°)
        const pitchBar = document.getElementById('imuPitchBar');
        if (Math.abs(pitch) > 10) {
          pitchBar.style.background = 'linear-gradient(90deg,#b71c1c,#f44336)';
        } else {
          pitchBar.style.background = 'linear-gradient(90deg,#e65100,#ff9800)';
        }
      })();

      // Sensors
      function ss(dId, vId, v, tl, fl) {
        const dot = document.getElementById(dId), val = document.getElementById(vId);
        if (v===null||v===undefined){dot.className='dot dot-gray';val.textContent='—';return;}
        dot.className = v ? 'dot dot-red' : 'dot dot-green';
        val.textContent = v ? (tl||'YES') : (fl||'NO');
      }
      ss('dotLidar','valLidar',d.lidar_obstacle,'BLOCKED','CLEAR');
      ss('dotCam','valCam',d.camera_obstacle,'BLOCKED','CLEAR');
      ss('dotFused','valFused',d.fused_obstacle,'BLOCKED','CLEAR');
      ss('dotTunnel','valTunnel',d.tunnel_detected,'IN TUNNEL','NO');
      ss('dotObst','valObst',d.obstruction_active,'DODGING','NO');
      ss('dotPark','valPark',d.parking_complete,'DONE','NO');
      ss('dotSignage','valSignage',d.parking_sign_detected,'DETECTED','CLEAR');
      if (d.health_ok === null || d.health_ok === undefined) {
        document.getElementById('dotHealth').className = 'dot dot-gray';
        document.getElementById('valHealth').textContent = '—';
      } else if (d.health_ok) {
        document.getElementById('dotHealth').className = 'dot dot-green';
        document.getElementById('valHealth').textContent = 'OK';
      } else {
        document.getElementById('dotHealth').className = 'dot dot-red';
        document.getElementById('valHealth').textContent = 'STALE';
      }
      const stale = Array.isArray(d.health_stale) ? d.health_stale : (Array.isArray(d.stale_streams) ? d.stale_streams : []);
      const staleEl = document.getElementById('valStale');
      if (stale.length === 0) {
        staleEl.textContent = 'NONE';
        staleEl.style.color = '#9bd69b';
      } else {
        staleEl.textContent = stale.slice(0, 3).join(', ') + (stale.length > 3 ? ' +' + (stale.length - 3) : '');
        staleEl.style.color = '#f38ba8';
      }

      // Warning Flash Overlay (if blocked in AUTO)
      const isBlocked = d.fused_obstacle; // Use fused for reliable alarm
      const overlay = document.getElementById('warning-overlay');
      if (overlay) {
        if (d.auto_mode && isBlocked) overlay.classList.add('active');
        else overlay.classList.remove('active');
      }

      const gD=document.getElementById('dotGate'),gV=document.getElementById('valGate');
      if(d.boom_gate===null){gD.className='dot dot-gray';gV.textContent='â€”';}
      else{gD.className=d.boom_gate?'dot dot-green':'dot dot-red';gV.textContent=d.boom_gate?'OPEN':'CLOSED';}

      // Lane
      document.getElementById('laneErr').textContent = d.lane_error.toFixed(3);
      document.getElementById('laneBar').style.width = Math.min(Math.max((d.lane_error+0.5)/1.0*100,0),100)+'%';
      document.getElementById('cmdLinX').textContent = d.cmd_lin_x.toFixed(3);
      document.getElementById('cmdAngZ').textContent = d.cmd_ang_z.toFixed(3);

      // Odom
      document.getElementById('odomDist').textContent = d.distance.toFixed(2);
      document.getElementById('odomSpeed').textContent = d.speed.toFixed(3)+' m/s';
      document.getElementById('odomX').textContent = (d.odom_x || 0).toFixed(2) + ' m';
      document.getElementById('odomY').textContent = (d.odom_y || 0).toFixed(2) + ' m';
      document.getElementById('odomYaw').textContent = ((d.odom_yaw || 0) * 180 / Math.PI).toFixed(1) + '°';

      // Speed & Selector
      document.getElementById('speedPct').textContent = d.speed_pct+'%';
      document.getElementById('speedBar').style.width = d.speed_pct+'%';
      // Update gear dots
      const gears = [25,40,60,100];
      gears.forEach((g,i) => {
        const dot = document.getElementById('gear'+i);
        if(dot) dot.classList.toggle('active', d.speed_pct >= g);
      });
      document.getElementById('ctrlState').textContent = d.ctrl_state_name;

      // Controller buttons â€” Updated to match user's specific mapping:
      // A=0, B=1, X=3, Y=4, LB=6, RB=7, Start=11
      const bm={0:'btnA',1:'btnB',3:'btnX',4:'btnY',6:'btnLB',7:'btnRB',8:'btnLT',9:'btnRT',11:'btnStart'};
      Object.values(bm).forEach(id=>document.getElementById(id).classList.remove('active'));
      if(d.buttons){
        d.buttons.forEach((v,i)=>{if(v&&bm[i])document.getElementById(bm[i]).classList.add('active');});
        const p=[];d.buttons.forEach((v,i)=>{if(v)p.push('btn['+i+']');});
        const db=document.getElementById('btnDebug');
        if(p.length){db.textContent='Active: '+p.join(', ');db.style.color='var(--success)';}
        else{db.textContent='No buttons pressed';db.style.color='var(--muted)';}
      }

      // D-pad (axes 6,7)
      if(d.axes&&d.axes.length>7){
        document.getElementById('btnLeft').classList.toggle('active',d.axes[6]>0.5);
        document.getElementById('btnRight').classList.toggle('active',d.axes[6]<-0.5);
        document.getElementById('btnUp').classList.toggle('active',d.axes[7]>0.5);
        document.getElementById('btnDown').classList.toggle('active',d.axes[7]<-0.5);
      }
      if(d.axes&&d.axes.length>3){
        document.getElementById('joyValL').textContent='L: '+d.axes[0].toFixed(1)+', '+d.axes[1].toFixed(1);
        document.getElementById('joyValR').textContent='R: '+d.axes[2].toFixed(1)+', '+d.axes[3].toFixed(1);
        
        let lx = 50 + (d.axes[0] * -50);
        let ly = 50 + (d.axes[1] * -50);
        let rx = 50 + (d.axes[2] * -50);
        let ry = 50 + (d.axes[3] * -50);
        
        const dl = document.getElementById('joyDotL');
        const dr = document.getElementById('joyDotR');
        
        dl.style.left = lx + '%';
        dl.style.top = ly + '%';
        dl.classList.toggle('active', Math.abs(d.axes[0]) > 0.05 || Math.abs(d.axes[1]) > 0.05);
        
        dr.style.left = rx + '%';
        dr.style.top = ry + '%';
        dr.classList.toggle('active', Math.abs(d.axes[2]) > 0.05 || Math.abs(d.axes[3]) > 0.05);
      }

      // ── Record & Playback State ──
      const rpState = d.rp_state || 'IDLE';
      const bufSize = d.rp_buffer_size || 0;
      const pbIdx = d.rp_playback_index || 0;

      const badge = document.getElementById('rpStateBadge');
      if (badge) {
        badge.textContent = rpState;
        badge.className = 'rp-state-badge ' + rpState.toLowerCase();
      }

      const bufSizeEl = document.getElementById('rpBufferSize');
      if (bufSizeEl) bufSizeEl.textContent = bufSize;
      
      const durEl = document.getElementById('rpDuration');
      if (durEl) durEl.textContent = (bufSize * 0.05).toFixed(1);

      const btnRec = document.getElementById('rpBtnRecord');
      const btnStop = document.getElementById('rpBtnStop');
      const btnPlay = document.getElementById('rpBtnPlay');
      const btnSave = document.getElementById('rpBtnSave');

      if (btnRec) {
        btnRec.classList.toggle('active', rpState === 'RECORDING');
        btnRec.disabled = (rpState === 'PLAYBACK');
      }
      if (btnPlay) {
        btnPlay.classList.toggle('active', rpState === 'PLAYBACK');
        btnPlay.disabled = (rpState === 'RECORDING' || bufSize === 0);
      }
      if (btnStop) {
        btnStop.disabled = (rpState === 'IDLE');
      }
      if (btnSave) {
        btnSave.disabled = (rpState !== 'IDLE' || bufSize === 0);
      }

      const progressTrack = document.getElementById('rpProgress');
      const progressFill = document.getElementById('rpProgressFill');
      if (progressTrack && progressFill) {
        if (rpState === 'PLAYBACK' && bufSize > 0) {
          progressTrack.style.display = 'block';
          const pct = Math.min(100, (pbIdx / bufSize) * 100);
          progressFill.style.width = pct + '%';
        } else {
          progressTrack.style.display = 'none';
          progressFill.style.width = '0%';
        }
      }
    })
    .catch(()=>{
      document.getElementById('connDot').style.background='#f44336';
      document.getElementById('connText').textContent='Disconnected';
    });
}
// ===== PARAMETER TUNING (curated) =====
const PARAM_TIPS = {
  // Traffic light
  red_h_low1:'Red hue range 1 lower bound (HSV)', red_h_high1:'Red hue range 1 upper bound',
  red_h_low2:'Red hue range 2 lower bound (wrap)', red_h_high2:'Red hue range 2 upper bound',
  yellow_h_low:'Yellow hue lower', yellow_h_high:'Yellow hue upper',
  green_h_low:'Green hue lower', green_h_high:'Green hue upper',
  sat_min:'Min saturation to count as color', val_min:'Min brightness to count as color',
  min_circle_radius:'Smallest circle to detect', max_circle_radius:'Largest circle to detect',
  min_pixel_count:'Min colored pixels to trigger', required_confidence:'Consecutive detections required',
  resize_width:'Resize width for CV speed', heartbeat_sec:'Publish heartbeat period',
  // Line follower (MDPI-enhanced scanline)
  n_scanlines:'Number of horizontal scanlines to sample', min_valid_scanlines:'Min scanlines for confident lock',
  min_line_width_px:'Min white region width in pixels (noise filter)',
  crop_ratio_base:'Bottom crop ratio — how much of the frame is road',
  white_threshold:'Fixed gray threshold for binary (used when use_otsu=false)',
  use_otsu:'Use Otsu auto-threshold instead of fixed white_threshold',
  invert_binary:'Invert binary image — detect dark lane (true) or white borders (false)',
  morph_open_size:'Morphological OPEN kernel size — removes small noise blobs (0=disable, 3=default)',
  morph_close_size:'Morphological CLOSE kernel size — fills small gaps in lines (0=disable, 5=default)',
  search_radius_px:'Blob-to-expected-position match radius in pixels',
  clahe_enabled:'Enable CLAHE adaptive lighting normalization', clahe_clip_limit:'CLAHE contrast clip limit',
  // IPM (Birds Eye View)
  ipm_enabled:'Enable Birds Eye View perspective warp (MDPI)', ipm_top_width_ratio:'Narrow end of trapezoid (0.2-0.5, lower=stronger warp)',
  ipm_bottom_width_ratio:'Wide end of trapezoid (usually 1.0)',
  // Kalman filter
  kalman_enabled:'Use Kalman filter instead of EMA for lane smoothing',
  kalman_process_noise:'Q — how much the filter trusts its model (lower=smoother)',
  kalman_measurement_noise:'R — how much the filter trusts measurements (lower=more reactive)',
  // Legacy EMA
  smoothing_alpha:'EMA smoothing (0=smooth, 1=raw) — used when Kalman disabled', dead_zone:'Tolerance threshold — ignore error below this',
  hold_error_frames:'Frames to hold last known error when lines lost',
  error_decay_rate:'Per-frame decay for held error (0.92 = halves in ~9 frames)',
  debug_print_rate:'Seconds between console debug prints',
  // Auto driver
  steering_gain:'Lane steering gain (legacy)', forward_speed:'Max forward speed (m/s) — on a straight',
  stale_timeout:'Seconds before module data is stale', dist_lap_complete:'Distance after green to mark lap',
  enable_subsumption_obstacle:'Enable fused obstacle reverse adjust', max_odom_speed:'Ignore odom speed spikes above this',
  min_state_dwell_sec:'Minimum hold time for non-emergency behavior switches',
  publish_loop_stats:'Enable loop timing diagnostics stream',
  // PID Steering
  pid_kp:'[PID] Proportional gain — how hard to steer. Too high → oscillation/weaving',
  pid_ki:'[PID] Integral gain — corrects long-term drift. Keep very small (0.0–0.05)',
  pid_kd:'[PID] Derivative gain — dampens oscillation. Too high → jittery steering',
  pid_integral_max:'[PID] Anti-windup clamp for I term. Prevents integral runaway in long turns',
  speed_error_scale:'Adaptive speed: higher = robot slows more in turns (try 1.0–2.5)',
  min_turn_speed:'Adaptive speed: minimum speed multiplier in a sharp turn (0.4 = 40% of max)',
  lane_steer_slew:'Max steering change per second — lower = smoother but slower response (Cytron accel limit)',
  // Command safety
  publish_hz:'Safety controller publish loop frequency',
  cmd_timeout:'Max age for raw auto commands before forced zero',
  max_linear_speed:'Clamp for linear auto command speed',
  max_angular_speed:'Clamp for angular auto command speed',
  max_linear_accel:'Linear acceleration slew limit',
  max_angular_accel:'Angular acceleration slew limit',
  deadband_linear:'Zero small linear command noise',
  deadband_angular:'Zero small angular command noise',
  // Tunnel wall follower (RANSAC enhanced)
  target_center_dist:'Offset from center between walls (0=centered)',
  kp_heading:'Proportional gain for heading alignment (wall angle correction)',
  kd_heading:'Derivative gain for heading alignment (dampens heading oscillation)',
  ransac_threshold:'RANSAC inlier distance threshold in meters (lower=stricter fit)',
  ransac_iterations:'RANSAC max iterations (50 is fine for <100 LiDAR pts)',
  tunnel_hysteresis_frames:'Consecutive frames required before toggling tunnel on/off (prevents flicker)',
  min_wall_points:'Min LiDAR points on each side to consider a wall present',
  max_wall_dist:'Ignore LiDAR points beyond this distance (m)',
  // Boom gate
  min_detect_dist:'Closest gate detection (m)', max_detect_dist:'Farthest gate detection (m)',
  angle_window:'Forward arc width (rad)', min_gate_points:'Min LiDAR points for gate',
  distance_variance_max:'Max spread of gate points', lidar_angle_offset:'LiDAR mount rotation (rad)',
  hysteresis:'Consecutive frames before state change',
  // Obstruction
  detect_dist:'Start dodging at this distance (m)', clear_dist:'Consider clear beyond this (m)',
  front_angle:'Front detection arc (rad)', side_angle_min:'Side arc start (rad)', side_angle_max:'Side arc end (rad)',
  steer_speed:'Speed while steering around (m/s)', steer_angular:'Turn rate while dodging (rad/s)',
  pass_speed:'Speed while passing alongside (m/s)', pass_duration:'Seconds driving alongside',
  steer_back_duration:'Seconds steering back to lane', steer_away_duration:'Seconds steering away',
  // Parking
  parallel_forward_dist:'Drive past slot distance (m)', parallel_reverse_dist:'Reverse into slot distance (m)',
  parallel_steer_angle:'Steering rate during reverse (rad/s)', perp_turn_angle:'Turn angle into slot (rad)',
  perp_forward_dist:'Drive into slot distance (m)', park_wait_time:'Seconds to wait in slot',
  drive_speed:'Parking drive speed (m/s)', reverse_speed:'Parking reverse speed (m/s)',
  signboard_min_area:'Min contour area for parking sign', signboard_resize_width:'Resize width for sign detection',
  // Camera obstacle
  edge_threshold:'Edge pixel ratio to trigger (0.0-1.0)', canny_low:'Canny lower threshold',
  canny_high:'Canny upper threshold', blur_kernel:'Gaussian blur kernel (odd number)',
  hysteresis_on:'Frames to confirm obstacle', hysteresis_off:'Frames to confirm clear',
  // Dashboard
  use_hw_odom:'Use hardware encoder odometry', freshness_stale_sec:'Dashboard stale threshold in seconds',
  sim_odom_scale:'Scale for simulated odometry distance', hw_odom_scale:'Scale for hardware odometry distance',
  hw_odom_yaw_scale:'Scale for hardware odometry yaw',
  // Servo controller
  servo_center:'Servo neutral angle (90=default). Adjust if robot drifts left/right when steering is centered',
  servo_range_left:'Left steering range from center (50=default). Increase for sharper left turns',
  servo_range_right:'Right steering range from center (50=default). Increase for sharper right turns',
  servo_steer_id:'Servo channel for steering (4=default)',
  auto_cmd_timeout:'Auto cmd stream timeout before forcing manual stop',
  unlock_requires_neutral:'Require neutral sticks after unlock before driving',
  unlock_neutral_threshold:'Neutral stick threshold for unlock gate',
  ticks_per_meter:'Encoder ticks per meter', odom_distance_scale:'Extra distance calibration multiplier',
  drive_motor_index:'Which motor encoder channel to use (0=FL 1=FR 2=RL 3=RR)',
  // Challenge sequencing
  t_post_obstacle_sec:'Lane-follow delay after obstacle clears before entering roundabout (sec)',
  t_roundabout_sec:'Time to traverse the roundabout arc before exiting (sec)',
  odom_yaw_scale:'Extra yaw calibration multiplier', encoder_jump_threshold:'Reject tick jumps above this',
  max_linear_velocity:'Clamp linear odom speed', max_angular_velocity:'Clamp angular odom speed',
  odom_reverse_polarity:'Invert encoder odom sign if needed', odom_velocity_deadband:'Zero odom near standstill',
  // Health monitor
  publish_period:'Health publish period (s)', timeout_perception:'Perception timeout (s)',
  timeout_state:'State timeout (s)', timeout_control:'Control timeout (s)',
  timeout_odom:'Odometry timeout (s)', timeout_joy:'Joystick timeout (s)',
  show_debug:'Publish annotated debug frame', print_debug:'Enable console debug printing',
  // Signage detector (BPU)
  model_path:'Path to compiled BPU .bin model file',
  conf_threshold:'Global YOLO confidence fallback (0.0–1.0)',
  iou_threshold:'NMS IoU threshold (0.0–1.0)',
  min_parking_sign_width:'Min pixel width for parking sign trigger (0 = disabled)',
  // Per-class confidence thresholds
  thresh_bumper:'Confidence threshold for Bumper_signboard (class 0)',
  thresh_hill:'Confidence threshold for Hill_signboard (class 1)',
  thresh_obstacle:'Confidence threshold for Obstacle_signboard (class 2)',
  thresh_parallelp:'Confidence threshold for ParallelP_signboard (class 3)',
  thresh_perpendp:'Confidence threshold for PerpendP_signboard (class 4)',
  thresh_roundabout:'Confidence threshold for Roundabout_signboard (class 5)',
  thresh_tl_green:'Confidence threshold for Traffic_Green (class 6)',
  thresh_tl_red:'Confidence threshold for Traffic_Red (class 7)',
  thresh_tl_generic:'Confidence threshold for Trafficlight_signboard generic (class 8)',
  // Per-class bounding box colors (format: "B,G,R" integers 0–255)
  color_bumper:'Bounding box color for Bumper_signboard — format "B,G,R" e.g. 0,140,255',
  color_hill:'Bounding box color for Hill_signboard — format "B,G,R" e.g. 180,0,255',
  color_obstacle:'Bounding box color for Obstacle_signboard — format "B,G,R" e.g. 0,255,255',
  color_parallelp:'Bounding box color for ParallelP_signboard — format "B,G,R" e.g. 255,0,0',
  color_perpendp:'Bounding box color for PerpendP_signboard — format "B,G,R" e.g. 0,100,0',
  color_roundabout:'Bounding box color for Roundabout_signboard — format "B,G,R" e.g. 255,255,255',
  color_tl_green:'Bounding box color for Traffic_Green — format "B,G,R" e.g. 0,255,0',
  color_tl_red:'Bounding box color for Traffic_Red — format "B,G,R" e.g. 0,0,255',
  color_tl_generic:'Bounding box color for Trafficlight_signboard generic — format "B,G,R" e.g. 255,255,0'
};
const PARAM_GROUPS = [
  { node: 'line_follower_camera', label: 'Line Follower', params: [
    'n_scanlines','min_valid_scanlines','min_line_width_px',
    'crop_ratio_base','search_radius_px',
    'white_threshold','use_otsu','invert_binary',
    'morph_open_size','morph_close_size',
    'clahe_enabled','clahe_clip_limit',
    'ipm_enabled','ipm_top_width_ratio','ipm_bottom_width_ratio',
    'kalman_enabled','kalman_process_noise','kalman_measurement_noise',
    'smoothing_alpha','dead_zone','hold_error_frames','error_decay_rate',
    'resize_width','print_debug','debug_print_rate','show_debug'
  ]},
  { node: 'auto_driver', label: 'Auto Driver', params: [
    'forward_speed','stale_timeout',
    'dist_lap_complete','enable_subsumption_obstacle','max_odom_speed',
    'min_state_dwell_sec','publish_loop_stats',
    'pid_kp','pid_ki','pid_kd','pid_integral_max',
    'speed_error_scale','min_turn_speed','lane_steer_slew',
    't_post_obstacle_sec','t_roundabout_sec'
  ]},
  { node: 'cmd_safety_controller', label: 'Cmd Safety', params: [
    'publish_hz','cmd_timeout','max_linear_speed','max_angular_speed',
    'max_linear_accel','max_angular_accel','deadband_linear','deadband_angular','publish_loop_stats'
  ]},
  { node: 'boom_gate_detector', label: 'Boom Gate', params: [
    'min_detect_dist','max_detect_dist','angle_window',
    'min_gate_points','distance_variance_max','lidar_angle_offset','hysteresis','heartbeat_sec'
  ]},
  { node: 'tunnel_wall_follower', label: 'Tunnel', params: [
    'target_center_dist','forward_speed','kp','kd','kp_heading','kd_heading','max_angular',
    'left_angle_min','left_angle_max','right_angle_min','right_angle_max',
    'min_wall_points','max_wall_dist','lidar_angle_offset',
    'ransac_threshold','ransac_iterations','tunnel_hysteresis_frames','heartbeat_sec'
  ]},
  { node: 'obstruction_avoidance', label: 'Obstruction', params: [
    'detect_dist','clear_dist','front_angle','side_angle_min','side_angle_max',
    'steer_speed','steer_angular','pass_speed','pass_duration',
    'steer_back_duration','steer_away_duration','lidar_angle_offset'
  ]},
  { node: 'parking_controller', label: 'Parking', params: [
    'parallel_forward_dist','parallel_reverse_dist','parallel_steer_angle',
    'perp_turn_angle','perp_forward_dist','park_wait_time',
    'drive_speed','reverse_speed','signboard_min_area','signboard_resize_width'
  ]},
  { node: 'obstacle_avoidance_camera', label: 'Camera Obstacle', params: [
    'edge_threshold','canny_low','canny_high','blur_kernel',
    'hysteresis_on','hysteresis_off','resize_width','heartbeat_sec','show_debug'
  ]},
  { node: 'obstacle_avoidance_node', label: 'LiDAR Obstacle', params: [
    'min_obstacle_distance','heartbeat_sec'
  ]},
  { node: 'servo_controller', label: 'Servo/Odom', params: [
    'servo_center','servo_range_left','servo_range_right','servo_steer_id',
    'joy_timeout','auto_cmd_timeout','unlock_requires_neutral','unlock_neutral_threshold',
    'ticks_per_meter','drive_motor_index','odom_distance_scale','odom_yaw_scale',
    'encoder_jump_threshold','max_linear_velocity','max_angular_velocity',
    'wheel_base','steering_max_deg','odom_vel_alpha','odom_velocity_deadband',
    'odom_reverse_polarity','publish_loop_stats'
  ]},
  { node: 'dashboard', label: 'Dashboard', params: [
    'use_hw_odom','freshness_stale_sec','sim_odom_scale','hw_odom_scale','hw_odom_yaw_scale'
  ]},
  { node: 'health_monitor', label: 'Health Monitor', params: [
    'publish_period','timeout_perception','timeout_state',
    'timeout_control','timeout_odom','timeout_joy'
  ]},
  { node: 'signage_detector', label: 'Signage Detector (BPU)', params: [
    'model_path','conf_threshold','iou_threshold',
    'min_parking_sign_width','heartbeat_sec','show_debug',
    'thresh_bumper','thresh_hill','thresh_obstacle',
    'thresh_parallelp','thresh_perpendp','thresh_roundabout',
    'thresh_tl_green','thresh_tl_red','thresh_tl_generic',
    'color_bumper','color_hill','color_obstacle',
    'color_parallelp','color_perpendp','color_roundabout',
    'color_tl_green','color_tl_red','color_tl_generic'
  ]},
];

function toggleNodeParams(headerEl, nodeName) {
  const body = headerEl.nextElementSibling;
  const opening = !body.classList.contains('open');
  body.classList.toggle('open');
  if (opening) {
    const group = PARAM_GROUPS.find(g => g.node === nodeName);
    if (group) {
      group.params.forEach(p => {
        getParam(nodeName, p, true);
      });
    }
  }
}

function buildParamUI() {
  const c = document.getElementById('paramContainer');
  c.innerHTML = PARAM_GROUPS.map(g => {
    const rows = g.params.map(p => {
      const tip = PARAM_TIPS[p] || '';
      return `<div class="param-row">
        <span class="param-name" ${tip ? 'title="'+tip+'"' : ''}>${p}</span>
        <input class="param-val" id="pv_${g.node}_${p}" placeholder="—" />
        <button class="param-get-btn" onclick="getParam('${g.node}','${p}')">Get</button>
        <button class="param-set-btn" onclick="setParam('${g.node}','${p}')">Set</button>
        <span class="param-status" id="ps_${g.node}_${p}"></span>
      </div>`;
    }).join('');
    return `<div class="param-node-block">
      <div class="param-node-header" onclick="toggleNodeParams(this, '${g.node}')">
        <span class="param-node-name">${g.label}</span>
        <span class="param-node-count">${g.params.length} params</span>
      </div>
      <div class="param-node-body">${rows}</div>
    </div>`;
  }).join('');
}

async function getParam(node, param, isInitialLoad=false) {
  const status = document.getElementById('ps_' + node + '_' + param);
  const input = document.getElementById('pv_' + node + '_' + param);
  if (status && !isInitialLoad) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/get_param?node=' + encodeURIComponent(node) + '&param=' + encodeURIComponent(param));
    const d = await r.json();
    if (d.ok) {
      if (input) {
        input.value = d.value;
        if (isInitialLoad) {
          const defVal = d.default !== undefined ? d.default : d.value;
          input.placeholder = `Def: ${defVal}`;
          const pName = input.parentElement.querySelector('.param-name');
          if (pName) {
            pName.innerHTML = `${param} <span style="color:#555;font-size:0.85em;">(def: ${defVal})</span>`;
          }
        }
      }
      if (status && !isInitialLoad) { status.className = 'param-status ok'; status.textContent = 'âœ“'; }
    } else {
      if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = d.error || 'Not found'; }
    }
  } catch(e) {
    if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  if (!isInitialLoad) {
    setTimeout(() => { if(status) status.textContent = ''; }, 3000);
  }
}

async function setParam(node, param) {
  const input = document.getElementById('pv_' + node + '_' + param);
  const status = document.getElementById('ps_' + node + '_' + param);
  if (!input || !input.value) { if(status){status.className='param-status err';status.textContent='Empty';} return; }
  if (status) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/set_param', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({node: node, param: param, value: input.value})
    });
    const d = await r.json();
    if (d.ok) {
      if (status) { status.className = 'param-status ok'; status.textContent = 'âœ“ Set'; }
    } else {
      if (status) { status.className = 'param-status err'; status.textContent = d.error || 'Failed'; }
    }
  } catch(e) {
    if (status) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  setTimeout(() => { if(status) status.textContent = ''; }, 3000);
}

async function saveDefaults() {
  if (!confirm('Save ALL current runtime parameters as the new defaults in params.yaml?\\n\\nThis will overwrite the file on disk. You will need to rebuild (colcon build) for the changes to take effect on next launch.')) {
    return;
  }
  const btn = document.getElementById('saveDefaultsBtn');
  const status = document.getElementById('saveDefaultsStatus');
  btn.className = 'param-save-defaults-btn saving';
  btn.textContent = 'â³ Saving...';
  status.textContent = '';
  try {
    const r = await fetch('/api/save_defaults', { method: 'POST' });
    const d = await r.json();
    if (d.ok) {
      btn.className = 'param-save-defaults-btn success';
      btn.textContent = 'âœ“ Saved!';
      status.style.color = 'var(--success)';
      status.textContent = d.msg || `${d.updated} params updated`;
    } else {
      btn.className = 'param-save-defaults-btn error';
      btn.textContent = 'âœ— Failed';
      status.style.color = 'var(--danger)';
      status.textContent = d.error || 'Unknown error';
    }
  } catch(e) {
    btn.className = 'param-save-defaults-btn error';
    btn.textContent = 'âœ— Error';
    status.style.color = 'var(--danger)';
    status.textContent = 'Network error';
  }
  setTimeout(() => {
    btn.className = 'param-save-defaults-btn';
    btn.textContent = 'ðŸ’¾ Save Current as Default';
    status.textContent = '';
  }, 5000);
}

buildParamUI();

// Parameters are now fetched on-demand when expanding the sections in the UI.

setInterval(update, 200);
update();

// â”€â”€ LiDAR 2D Visualization â”€â”€
(function(){
  const canvas = document.getElementById('lidarCanvas');
  if(!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const CX = W/2, CY = H/2;
  const SCALE = 200;

  function drawLidar(points, tunnelDetected){
    ctx.clearRect(0,0,W,H);
    // Grid circles
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1;
    [0.2, 0.4, 0.6].forEach(function(r){
      ctx.beginPath(); ctx.arc(CX, CY, r*SCALE, 0, Math.PI*2); ctx.stroke();
    });
    // Distance labels
    ctx.fillStyle = 'rgba(255,255,255,0.15)';
    ctx.font = '10px Inter, sans-serif';
    ctx.textAlign = 'left';
    [0.2, 0.4, 0.6].forEach(function(r){
      ctx.fillText(r.toFixed(1)+'m', CX+r*SCALE+2, CY-2);
    });
    // Tunnel detection windows
    ctx.globalAlpha = 0.08;
    ctx.fillStyle = '#4fc3f7';
    ctx.beginPath(); ctx.moveTo(CX, CY);
    ctx.arc(CX, CY, 0.6*SCALE, (-90-90)*Math.PI/180, (-90-30)*Math.PI/180);
    ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#ef5350';
    ctx.beginPath(); ctx.moveTo(CX, CY);
    ctx.arc(CX, CY, 0.6*SCALE, (-90+30)*Math.PI/180, (-90+90)*Math.PI/180);
    ctx.closePath(); ctx.fill();
    ctx.globalAlpha = 1.0;
    // Forward line
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.setLineDash([4,4]);
    ctx.beginPath(); ctx.moveTo(CX, CY); ctx.lineTo(CX, CY - 0.7*SCALE); ctx.stroke();
    ctx.setLineDash([]);
    // LiDAR points
    if(points && points.length > 0){
      points.forEach(function(p){
        var px = CX - p.y * SCALE;
        var py = CY - p.x * SCALE;
        var dist = Math.sqrt(p.x*p.x + p.y*p.y);
        if(dist < 0.3) ctx.fillStyle = '#ff5252';
        else if(dist < 0.5) ctx.fillStyle = '#ffd740';
        else ctx.fillStyle = '#69f0ae';
        ctx.beginPath(); ctx.arc(px, py, 2.5, 0, Math.PI*2); ctx.fill();
      });
    }
    // Centerline path (cyan dots + line)
    if(window._lastCenterline && window._lastCenterline.length > 1){
      ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 2; ctx.setLineDash([3,3]);
      ctx.beginPath();
      window._lastCenterline.forEach(function(p, i){
        var px = CX - p.y * SCALE;
        var py = CY - p.x * SCALE;
        if(i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke(); ctx.setLineDash([]);
      // Draw center dots
      ctx.fillStyle = '#00e5ff';
      window._lastCenterline.forEach(function(p){
        var px = CX - p.y * SCALE;
        var py = CY - p.x * SCALE;
        ctx.beginPath(); ctx.arc(px, py, 3.5, 0, Math.PI*2); ctx.fill();
      });
    }
    // Robot icon
    ctx.fillStyle = '#1e88e5'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(CX, CY - 10); ctx.lineTo(CX - 7, CY + 6); ctx.lineTo(CX + 7, CY + 6);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    // Labels
    ctx.fillStyle = 'rgba(255,255,255,0.3)';
    ctx.font = '9px Inter, sans-serif'; ctx.textAlign = 'center';
    ctx.fillText('FRONT', CX, 14);
    ctx.fillText('L', 12, CY+4);
    ctx.fillText('R', W-12, CY+4);
    // Status
    var el = document.getElementById('lidarStatus');
    if(tunnelDetected) el.innerHTML = '<span style="color:#40a02b;">â— TUNNEL</span>';
    else if(points && points.length > 0) el.innerHTML = '<span style="color:#1e66f5;">â— ' + points.length + ' pts</span>';
    else el.innerHTML = '<span style="color:#888;">â— No data</span>';
  }
  function fetchLidar(){
    fetch('/lidar_data').then(function(r){return r.json();}).then(function(d){
      // Store centerline for drawing
      window._lastCenterline = d.centerline || [];
      drawLidar(d.points || [], d.tunnel || false);
      // Show steer debug overlay
      var el = document.getElementById('lidarDebug');
      if(!el){
        el = document.createElement('div');
        el.id = 'lidarDebug';
        el.style.cssText = 'font:bold 11px JetBrains Mono,monospace; color:#fff; padding:6px 8px; position:absolute; bottom:8px; left:8px; right:8px; background:rgba(0,0,0,0.7); border-radius:8px; display:none;';
        canvas.parentElement.style.position = 'relative';
        canvas.parentElement.appendChild(el);
      }
      if(d.tunnel && d.angular_z !== undefined){
        var dir = d.angular_z > 0.01 ? '← LEFT' : (d.angular_z < -0.01 ? 'RIGHT →' : '↑ STRAIGHT');
        var color = Math.abs(d.angular_z) > 0.3 ? '#ff5252' : '#69f0ae';
        el.innerHTML = 'L:' + (d.left_dist||0).toFixed(2) + 'm  R:' + (d.right_dist||0).toFixed(2) + 'm  lat:' + (d.dist_error||0).toFixed(3) + '  <span style="color:'+color+'">ω:' + (d.angular_z||0).toFixed(2) + ' ' + dir + '</span>';
        el.style.display = 'block';
      } else {
        el.style.display = 'none';
      }
    }).catch(function(){});
  }
  setInterval(fetchLidar, 250);
  fetchLidar();
})();
</script>
</body>
</html>"""

# ======================== TEACH HTML Dashboard ========================
TEACH_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RISA-Bot &mdash; Record &amp; Playback</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700;800&family=Inter:wght@400;500;600;700;800&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #090d16;
    --surface: #111827;
    --surface2: #1f2937;
    --surface-hover: #2d3748;
    --text: #f3f4f6;
    --text-muted: #9ca3af;
    --accent: #3b82f6; /* Blue */
    --accent-hover: #2563eb;
    --success: #10b981; /* Emerald */
    --danger: #ef4444; /* Red */
    --warning: #f59e0b; /* Amber */
    --recording: #ef4444;
    --playback: #10b981;
    --idle: #6b7280;
    --radius: 16px;
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Inter', sans-serif;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* ===== HEADER ===== */
  header {
    padding: 16px 32px;
    background: var(--surface);
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 2px solid rgba(255,255,255,0.05);
    position: sticky;
    top: 0;
    z-index: 100;
  }
  header h1 {
    font-size: 1.4em;
    font-weight: 800;
    letter-spacing: -0.5px;
    color: #fff;
  }
  .nav-link {
    color: var(--accent);
    text-decoration: none;
    font-size: 0.85em;
    font-weight: 600;
    padding: 6px 16px;
    border: 1px solid rgba(59,130,246,0.3);
    border-radius: 8px;
    transition: all 0.2s;
  }
  .nav-link:hover { background: rgba(59,130,246,0.1); border-color: var(--accent); }
  .header-right { display: flex; align-items: center; gap: 16px; }
  .mode-pill {
    padding: 6px 14px;
    border-radius: 20px;
    font-weight: 700;
    font-size: 0.85em;
    letter-spacing: 0.5px;
    transition: all 0.3s;
  }
  .mode-pill.auto { background: rgba(16,185,129,0.2); color: var(--success); }
  .mode-pill.manual { background: rgba(239,68,68,0.2); color: var(--danger); }

  /* ===== 3-COLUMN LAYOUT ===== */
  .teach-grid {
    display: grid;
    grid-template-columns: 340px 1fr 380px;
    gap: 20px;
    padding: 20px;
    max-width: 1600px;
    margin: 0 auto;
  }
  @media (max-width: 1200px) {
    .teach-grid {
      grid-template-columns: 340px 1fr;
    }
  }
  @media (max-width: 900px) {
    .teach-grid {
      grid-template-columns: 1fr;
    }
  }

  /* ===== CARDS ===== */
  .card {
    background: var(--surface);
    border-radius: var(--radius);
    padding: 24px;
    border: 1px solid rgba(255,255,255,0.06);
    transition: border-color 0.3s, box-shadow 0.3s;
  }
  .card:hover {
    border-color: rgba(66,165,245,0.2);
    box-shadow: 0 8px 32px rgba(0,0,0,0.3);
  }
  .card h2 {
    font-size: 0.75em;
    text-transform: uppercase;
    letter-spacing: 3px;
    color: var(--text-muted);
    margin-bottom: 16px;
    font-weight: 700;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .left-col { display: flex; flex-direction: column; gap: 20px; }
  .middle-col { display: flex; flex-direction: column; gap: 20px; }
  .right-col { display: flex; flex-direction: column; gap: 20px; }

  /* ===== RECORD & PLAYBACK PANEL ===== */
  .rp-state-display {
    text-align: center;
    margin-bottom: 20px;
  }
  .rp-state-badge {
    display: inline-block;
    padding: 10px 28px;
    border-radius: 12px;
    font-size: 1.3em;
    font-weight: 800;
    letter-spacing: 2px;
    transition: all 0.4s;
  }
  .rp-state-badge.idle {
    background: rgba(107,114,128,0.15);
    color: var(--idle);
    border: 1px solid rgba(107,114,128,0.3);
  }
  .rp-state-badge.recording {
    background: rgba(239,68,68,0.15);
    color: var(--recording);
    border: 1px solid rgba(239,68,68,0.4);
    animation: recPulse 1.5s ease infinite;
  }
  @keyframes recPulse { 0%,100%{box-shadow:0 0 0 0 rgba(239,68,68,0.3)} 50%{box-shadow:0 0 20px 4px rgba(239,68,68,0.2)} }
  .rp-state-badge.playback {
    background: rgba(16,185,129,0.15);
    color: var(--playback);
    border: 1px solid rgba(16,185,129,0.4);
    animation: playPulse 1.5s ease infinite;
  }
  @keyframes playPulse { 0%,100%{box-shadow:0 0 0 0 rgba(16,185,129,0.3)} 50%{box-shadow:0 0 20px 4px rgba(16,185,129,0.2)} }

  /* Control Buttons */
  .rp-buttons {
    display: flex;
    gap: 12px;
    margin-bottom: 20px;
  }
  .rp-btn {
    flex: 1;
    padding: 14px 8px;
    border-radius: 12px;
    border: 2px solid transparent;
    font-size: 1em;
    font-weight: 700;
    cursor: pointer;
    transition: all 0.25s cubic-bezier(0.4,0,0.2,1);
    font-family: inherit;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
  }
  .rp-btn .icon { font-size: 1.6em; }
  .rp-btn .label { font-size: 0.75em; letter-spacing: 1px; text-transform: uppercase; }

  .rp-btn.record {
    background: rgba(239,68,68,0.1);
    color: var(--recording);
    border-color: rgba(239,68,68,0.3);
  }
  .rp-btn.record:hover { background: rgba(239,68,68,0.2); border-color: var(--recording); transform: translateY(-2px); }
  .rp-btn.record:active { transform: scale(0.96); }
  .rp-btn.record.active {
    background: var(--recording);
    color: #fff;
    border-color: var(--recording);
    box-shadow: 0 0 24px rgba(239,68,68,0.4);
  }

  .rp-btn.stop {
    background: rgba(245,158,11,0.1);
    color: var(--warning);
    border-color: rgba(245,158,11,0.3);
  }
  .rp-btn.stop:hover { background: rgba(245,158,11,0.2); border-color: var(--warning); transform: translateY(-2px); }
  .rp-btn.stop:active { transform: scale(0.96); }

  .rp-btn.play {
    background: rgba(16,185,129,0.1);
    color: var(--playback);
    border-color: rgba(16,185,129,0.3);
  }
  .rp-btn.play:hover { background: rgba(16,185,129,0.2); border-color: var(--playback); transform: translateY(-2px); }
  .rp-btn.play:active { transform: scale(0.96); }
  .rp-btn.play.active {
    background: var(--playback);
    color: #111;
    border-color: var(--playback);
    box-shadow: 0 0 24px rgba(16,185,129,0.4);
  }

  .rp-btn.save {
    background: rgba(59,130,246,0.1);
    color: var(--accent);
    border-color: rgba(59,130,246,0.3);
  }
  .rp-btn.save:hover { background: rgba(59,130,246,0.2); border-color: var(--accent); transform: translateY(-2px); }
  .rp-btn.save:active { transform: scale(0.96); }

  .rp-btn:disabled {
    opacity: 0.3;
    cursor: not-allowed;
    transform: none !important;
  }

  /* Buffer / Progress Info */
  .rp-info {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
    margin-bottom: 16px;
  }
  .rp-info-item {
    background: var(--surface2);
    border-radius: 10px;
    padding: 14px;
    text-align: center;
  }
  .rp-info-item .value {
    font-family: 'JetBrains Mono', monospace;
    font-size: 2em;
    font-weight: 800;
    color: var(--accent);
    line-height: 1.2;
  }
  .rp-info-item .label {
    font-size: 0.7em;
    color: var(--text-muted);
    text-transform: uppercase;
    letter-spacing: 1.5px;
    margin-top: 4px;
  }

  /* Progress Bar */
  .progress-track {
    height: 8px;
    background: var(--surface2);
    border-radius: 4px;
    overflow: hidden;
    margin-bottom: 16px;
  }
  .progress-fill {
    height: 100%;
    border-radius: 4px;
    background: linear-gradient(90deg, var(--accent), var(--playback));
    transition: width 0.15s ease;
    position: relative;
  }
  .progress-fill::after {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0; bottom: 0;
    background: linear-gradient(90deg, transparent 0%, rgba(255,255,255,0.2) 50%, transparent 100%);
    animation: shimmer 1.5s infinite;
  }
  @keyframes shimmer { 0%{transform:translateX(-100%)} 100%{transform:translateX(100%)} }

  /* ===== CAMERA ===== */
  .cam-panel {
    background: #000;
    border-radius: 16px;
    border: 1px solid rgba(255,255,255,0.08);
    overflow: hidden;
    box-shadow: 0 12px 40px rgba(0,0,0,0.4);
  }
  .cam-container {
    display: flex;
    justify-content: center;
    align-items: center;
    background: #050508;
    min-height: 240px;
  }
  .cam-container img {
    width: 100%;
    height: 100%;
    object-fit: contain;
  }
  .cam-controls {
    display: flex; gap: 6px; padding: 10px; background: rgba(255,255,255,0.03);
  }
  .cam-btn {
    flex: 1; padding: 10px; border-radius: 8px; font-size: 0.85em; font-weight: 700;
    background: rgba(0,0,0,0.3); border: 1px solid rgba(255,255,255,0.08); color: #666;
    cursor: pointer; transition: all 0.2s; font-family: inherit;
  }
  .cam-btn.active { background: var(--accent); color: #fff; border-color: var(--accent); }
  .cam-btn:hover:not(.active) { background: rgba(255,255,255,0.05); color: #aaa; }

  /* ===== ODOMETRY / DATA CARDS ===== */
  .data-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
  }
  .data-card {
    background: var(--surface2);
    border-radius: 12px;
    padding: 16px;
    text-align: center;
  }
  .data-card h3 {
    font-size: 0.6em;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: var(--text-muted);
    margin-bottom: 6px;
    font-weight: 700;
  }
  .data-val {
    font-family: 'JetBrains Mono', monospace;
    font-size: 1.8em;
    font-weight: 800;
    color: var(--accent);
    line-height: 1.2;
  }
  .data-val.green { color: var(--success); }
  .data-val.yellow { color: var(--warning); }
  .data-unit { font-size: 0.35em; color: var(--text-muted); margin-left: 4px; }

  /* ===== STATUS DOT ===== */
  .status-dot {
    display: inline-block;
    width: 8px; height: 8px;
    border-radius: 50%;
    margin-right: 6px;
    vertical-align: middle;
  }
  .status-dot.live { background: var(--success); animation: pulse 2s infinite; }
  .status-dot.stale { background: var(--warning); }
  .status-dot.offline { background: #555; }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }

  .reset-btn {
    margin-top: 10px;
    padding: 8px 24px;
    background: var(--surface2);
    color: #aaa;
    border: 1px solid rgba(255,255,255,0.1);
    border-radius: 20px;
    cursor: pointer;
    font-size: 0.8em;
    font-weight: 600;
    transition: all 0.2s;
    font-family: inherit;
  }
  .reset-btn:hover { background: var(--surface-hover); color: #fff; border-color: rgba(255,255,255,0.2); }

  /* ===== RECORDING MANAGER ===== */
  .active-parking-info {
    background: rgba(16, 185, 129, 0.08);
    border: 1px solid rgba(16, 185, 129, 0.2);
    border-radius: 8px;
    padding: 10px 14px;
    margin-bottom: 16px;
    font-size: 0.85em;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .active-parking-info .badge {
    background: var(--success);
    color: #111;
    font-weight: 700;
    padding: 2px 8px;
    border-radius: 12px;
    font-size: 0.85em;
  }
  .recordings-list {
    display: flex;
    flex-direction: column;
    gap: 12px;
    max-height: 340px;
    overflow-y: auto;
    padding-right: 4px;
  }
  .recordings-list::-webkit-scrollbar {
    width: 6px;
  }
  .recordings-list::-webkit-scrollbar-track {
    background: transparent;
  }
  .recordings-list::-webkit-scrollbar-thumb {
    background: rgba(255,255,255,0.1);
    border-radius: 3px;
  }
  .recordings-list::-webkit-scrollbar-thumb:hover {
    background: rgba(255,255,255,0.2);
  }
  .no-recordings {
    color: var(--text-muted);
    font-style: italic;
    font-size: 0.9em;
    text-align: center;
    padding: 20px 0;
  }
  .recording-item {
    background: var(--surface2);
    border: 1px solid rgba(255,255,255,0.05);
    border-radius: 10px;
    padding: 12px 14px;
    cursor: pointer;
    transition: all 0.2s ease;
    display: flex;
    flex-direction: column;
    gap: 10px;
  }
  .recording-item:hover {
    border-color: rgba(66, 165, 245, 0.3);
    background: var(--surface-hover);
    transform: translateY(-1px);
  }
  .recording-item.selected {
    border-color: var(--accent);
    background: rgba(59, 130, 246, 0.08);
  }
  .recording-item.active-parking {
    border-left: 4px solid var(--success);
  }
  .recording-item.current {
    box-shadow: 0 0 10px rgba(59, 130, 246, 0.15);
  }
  .rec-name-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 8px;
  }
  .rec-name {
    font-weight: 700;
    font-size: 0.9em;
    color: var(--text);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .star-badge {
    background: rgba(16, 185, 129, 0.15);
    color: var(--success);
    font-size: 0.7em;
    font-weight: 700;
    padding: 2px 6px;
    border-radius: 4px;
    border: 1px solid rgba(16, 185, 129, 0.3);
  }
  .load-badge {
    background: rgba(59, 130, 246, 0.15);
    color: var(--accent);
    font-size: 0.7em;
    font-weight: 700;
    padding: 2px 6px;
    border-radius: 4px;
    border: 1px solid rgba(59, 130, 246, 0.3);
  }
  .rec-details {
    display: flex;
    flex-wrap: wrap;
    gap: 10px;
    font-size: 0.75em;
    color: var(--text-muted);
  }
  .rec-date {
    width: 100%;
    margin-top: 2px;
    color: #666;
  }
  .rec-actions {
    display: flex;
    gap: 6px;
    justify-content: flex-end;
  }
  .action-btn {
    padding: 6px 10px;
    border-radius: 6px;
    font-size: 0.75em;
    font-weight: 600;
    cursor: pointer;
    border: 1px solid transparent;
    transition: all 0.15s ease;
    background: rgba(255,255,255,0.05);
    color: var(--text);
  }
  .action-btn:hover { background: rgba(255,255,255,0.1); }
  .action-btn.load:hover { border-color: var(--accent); color: var(--accent); background: rgba(59, 130, 246, 0.05); }
  .action-btn.play:hover { border-color: var(--success); color: var(--success); background: rgba(16, 185, 129, 0.05); }
  .action-btn.set-active:hover { border-color: var(--warning); color: var(--warning); background: rgba(245, 158, 11, 0.05); }
  .action-btn.delete:hover { border-color: var(--danger); color: var(--danger); background: rgba(239, 68, 68, 0.05); }

  /* ===== TIMELINE VIEWER ===== */
  .timeline-card {
    display: flex;
    flex-direction: column;
  }
  .canvas-container {
    width: 100%;
    background: #070a13;
    border-radius: 10px;
    border: 1px solid rgba(255,255,255,0.04);
    padding: 10px;
  }
  .timeline-hint {
    text-align: center;
    font-size: 0.8em;
    color: var(--text-muted);
    margin-top: 8px;
  }

  /* ===== CONTROLLER HINTS ===== */
  .controller-hint {
    background: var(--surface2);
    border-radius: 10px;
    padding: 14px 16px;
    font-size: 0.8em;
    color: var(--text-muted);
    border: 1px dashed rgba(255,255,255,0.08);
  }
  .hint-row {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 4px 0;
  }
  .hint-key {
    display: inline-block;
    padding: 3px 10px;
    border-radius: 6px;
    background: rgba(59, 130, 246, 0.15);
    color: var(--accent);
    font-weight: 700;
    font-size: 0.9em;
    min-width: 50px;
    text-align: center;
  }

  /* ===== SAVE DIALOG MODAL ===== */
  .modal-overlay {
    position: fixed;
    top: 0; left: 0; width: 100vw; height: 100vh;
    background: rgba(0, 0, 0, 0.75);
    display: flex;
    justify-content: center;
    align-items: center;
    z-index: 1000;
    opacity: 0;
    pointer-events: none;
    transition: opacity 0.3s ease;
  }
  .modal-overlay.active {
    opacity: 1;
    pointer-events: auto;
  }
  .modal-content {
    background: var(--surface);
    border: 1px solid rgba(255,255,255,0.1);
    border-radius: var(--radius);
    padding: 30px;
    width: 100%;
    max-width: 420px;
    box-shadow: 0 20px 50px rgba(0,0,0,0.5);
    transform: scale(0.9);
    transition: transform 0.3s ease;
  }
  .modal-overlay.active .modal-content {
    transform: scale(1);
  }
  .modal-content h3 {
    margin-bottom: 15px;
    font-size: 1.3em;
    font-weight: 700;
  }
  .modal-content input {
    width: 100%;
    padding: 12px;
    border-radius: 8px;
    background: var(--bg);
    border: 1px solid rgba(255,255,255,0.15);
    color: var(--text);
    font-family: inherit;
    font-size: 1em;
    margin-bottom: 20px;
  }
  .modal-content input:focus {
    outline: none;
    border-color: var(--accent);
  }
  .modal-actions {
    display: flex;
    justify-content: flex-end;
    gap: 12px;
  }
  .modal-btn {
    padding: 10px 20px;
    border-radius: 8px;
    font-weight: 600;
    cursor: pointer;
    border: none;
    font-family: inherit;
  }
  .modal-btn.cancel {
    background: var(--surface2);
    color: var(--text);
  }
  .modal-btn.cancel:hover { background: var(--surface-hover); }
  .modal-btn.confirm {
    background: var(--accent);
    color: #fff;
  }
  .modal-btn.confirm:hover { background: var(--accent-hover); }
</style>
</head>
<body>

<!-- HEADER -->
<header>
  <h1>🤖 RISA-Bot / Record &amp; Playback</h1>
  <div class="header-right">
    <span class="mode-pill" id="modeBadge">WAITING</span>
    <a href="/" class="nav-link">← Dashboard</a>
  </div>
</header>

<!-- MAIN 3-COLUMN GRID -->
<div class="teach-grid">

  <!-- LEFT COLUMN: Camera + Odometry -->
  <div class="left-col">

    <!-- Camera Feed -->
    <div class="cam-panel">
      <div class="cam-container">
        <img id="camStream" src="/camera_feed?v=raw" alt="Camera Feed Offline"
             onerror="this.style.display='none'; document.getElementById('camOff').style.display='flex';"
             onload="this.style.display='block'; document.getElementById('camOff').style.display='none';"/>
        <div id="camOff" style="display:none; color:#555; width:100%; height:100%; align-items:center; justify-content:center; flex-direction:column; font-size:1.2em; font-weight:700; min-height:240px;">
          <div style="font-size:2em; margin-bottom:8px; opacity:0.4;">∅</div>
          <div>NO SIGNAL</div>
        </div>
      </div>
      <div class="cam-controls">
        <button class="cam-btn active" onclick="setCam('raw')" id="btn-raw">Raw</button>
        <button class="cam-btn" onclick="setCam('line_follower')" id="btn-line_follower">Lane</button>
        <button class="cam-btn" onclick="setCam('obstacle')" id="btn-obstacle">Obstacle</button>
        <button class="cam-btn" onclick="setCam('signage')" id="btn-signage">Signage</button>
      </div>
    </div>

  </div>

  <!-- MIDDLE COLUMN: Live Controls + Visual Timeline -->
  <div class="middle-col">

    <!-- Record & Playback Controls -->
    <div class="card">
      <h2>🎬 Recording Interface</h2>

      <div class="rp-state-display">
        <div class="rp-state-badge idle" id="rpStateBadge">IDLE</div>
      </div>

      <div class="rp-buttons">
        <button class="rp-btn record" id="rpBtnRecord" onclick="rpCmd('record')">
          <span class="icon">🔴</span>
          <span class="label">Record</span>
        </button>
        <button class="rp-btn stop" id="rpBtnStop" onclick="rpCmd('stop')">
          <span class="icon">⏹</span>
          <span class="label">Stop</span>
        </button>
        <button class="rp-btn play" id="rpBtnPlay" onclick="rpCmd('playback')">
          <span class="icon">▶</span>
          <span class="label">Play</span>
        </button>
        <button class="rp-btn save" id="rpBtnSave" onclick="showSaveModal()">
          <span class="icon">💾</span>
          <span class="label">Save</span>
        </button>
      </div>

      <div class="progress-track" id="rpProgress" style="display:none;">
        <div class="progress-fill" id="rpProgressFill" style="width:0%"></div>
      </div>

      <div class="rp-info">
        <div class="rp-info-item">
          <div class="value" id="rpBufferSize">0</div>
          <div class="label">Samples</div>
        </div>
        <div class="rp-info-item">
          <div class="value" id="rpDuration">0.0</div>
          <div class="label">Est. Duration (s)</div>
        </div>
      </div>
    </div>

    <!-- Timeline Viewer -->
    <div class="card timeline-card">
      <h2>📈 Recording Timeline</h2>
      <div class="canvas-container">
        <canvas id="timelineCanvas" style="width: 100%; height: 250px; display: block;"></canvas>
      </div>
      <div class="timeline-hint">
        <span style="color: #3b82f6;">■ Motor PWM (±255)</span> | <span style="color: #f97316;">■ Servo Angle (40–140)</span>
      </div>
    </div>

  </div>

  <!-- RIGHT COLUMN: Saved Recordings + Steering + Controller Mapping -->
  <div class="right-col">

    <!-- Recording Manager Card -->
    <div class="card">
      <h2>📂 Saved Recordings</h2>
      <div class="active-parking-info">
        <span>Active Parking:</span>
        <span class="badge" id="activeParkingBadge">None</span>
      </div>
      <div class="recordings-list" id="recordingsList">
        <div class="no-recordings">Loading recordings...</div>
      </div>
    </div>

    <!-- Steering Info -->
    <div class="card">
      <h2>Steering Status</h2>
      <div class="data-row">
        <div class="data-card">
          <h3>Lane Error</h3>
          <div class="data-val yellow" id="steerValBlock"><span id="steerErr">0.000</span></div>
        </div>
        <div class="data-card">
          <h3>Yaw</h3>
          <div class="data-val"><span id="logYaw">0.000</span><span class="data-unit">rad</span></div>
        </div>
      </div>
    </div>

    <!-- Controller Mapping -->
    <div class="card">
      <h2>🎮 Controller Mapping</h2>
      <div class="controller-hint">
        <div class="hint-row"><span class="hint-key">A</span> Record / Stop Recording</div>
        <div class="hint-row"><span class="hint-key">B</span> Save Recording (Quick)</div>
        <div class="hint-row"><span class="hint-key">X</span> Play / Stop Playback</div>
        <div class="hint-row"><span class="hint-key">Y</span> Auto/Manual Mode</div>
        <div class="hint-row"><span class="hint-key">D-Pad L/R</span> Cycle Recordings</div>
        <div class="hint-row"><span class="hint-key">D-Pad U/D</span> Speed ▲/▼</div>
      </div>
    </div>

  </div>
</div>

<!-- SAVE MODAL -->
<div class="modal-overlay" id="saveModal">
  <div class="modal-content">
    <h3>Save Recording</h3>
    <input type="text" id="saveNameInput" placeholder="Enter recording name" maxlength="32">
    <div class="modal-actions">
      <button class="modal-btn cancel" onclick="closeSaveModal()">Cancel</button>
      <button class="modal-btn confirm" onclick="confirmSave()">Save</button>
    </div>
  </div>
</div>

<script>
// ── Camera View Switching ──
function setCam(viewName) {
  fetch('/api/set_cam_view?view=' + viewName);
  document.getElementById('camStream').src = '/camera_feed?v=' + viewName + '&t=' + Date.now();
  ['raw', 'line_follower', 'obstacle', 'signage'].forEach(v => {
    const el = document.getElementById('btn-' + v);
    if (el) el.classList.toggle('active', v === viewName);
  });
}

// ── Reset Odometry ──
function resetOdom() {
  fetch('/api/reset_odom', {method:'POST'}).then(() => {
    document.getElementById('odomDist').textContent = '0.00';
    document.getElementById('posX').textContent = '0.00';
    document.getElementById('posY').textContent = '0.00';
  });
}

// ── Record/Playback Commands ──
function rpCmd(action, name = '') {
  fetch('/api/record_playback', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: action, name: name})
  }).catch(() => {});
}

// ── Save Modal Management ──
function showSaveModal() {
  const modal = document.getElementById('saveModal');
  const input = document.getElementById('saveNameInput');
  const now = new Date();
  const pad = (n) => String(n).padStart(2, '0');
  const timestamp = `rec_${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}`;
  input.value = timestamp;
  modal.classList.add('active');
  input.focus();
  input.select();
}

function closeSaveModal() {
  document.getElementById('saveModal').classList.remove('active');
}

function confirmSave() {
  const name = document.getElementById('saveNameInput').value.trim();
  if (!name) return;
  fetch('/api/record_playback', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: 'save', name: name})
  }).then(r => r.json())
    .then(res => {
      if (res.ok) {
        closeSaveModal();
      } else {
        alert('Error saving recording: ' + res.error);
      }
    });
}

// ── Recording Actions ──
function loadRec(name) {
  rpCmd('load', name);
}

function playRec(name) {
  fetch('/api/record_playback', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: 'load', name: name})
  }).then(() => {
    setTimeout(() => {
      rpCmd('playback');
    }, 150);
  });
}

function setActiveRec(name) {
  rpCmd('set_active', name);
}

function deleteRec(name) {
  if (confirm(`Are you sure you want to delete "${name}"?`)) {
    rpCmd('delete', name);
  }
}

function escapeHtml(str) {
  if (!str) return '';
  return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}

// ── Timeline Canvas Drawing ──
let loadedRecordingSamples = [];
let loadedRecordingName = '';

function drawTimeline(samples, name) {
  const canvas = document.getElementById('timelineCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  
  const width = canvas.clientWidth * 2;
  const height = canvas.clientHeight * 2;
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  
  ctx.clearRect(0, 0, width, height);
  
  if (!samples || samples.length === 0) {
    ctx.fillStyle = '#6b7280';
    ctx.font = '24px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('No sample data loaded', width / 2, height / 2);
    return;
  }
  
  const paddingLeft = 70;
  const paddingRight = 30;
  const paddingTop = 30;
  const paddingBottom = 40;
  const chartWidth = width - paddingLeft - paddingRight;
  const chartHeight = height - paddingTop - paddingBottom;
  
  // Draw Background Grid
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  ctx.lineWidth = 2;
  const numGridLines = 4;
  for (let i = 0; i <= numGridLines; i++) {
    const y = paddingTop + (chartHeight / numGridLines) * i;
    ctx.beginPath();
    ctx.moveTo(paddingLeft, y);
    ctx.lineTo(width - paddingRight, y);
    ctx.stroke();
  }
  
  // Draw Y Labels
  ctx.fillStyle = '#9ca3af';
  ctx.font = '18px JetBrains Mono';
  ctx.textAlign = 'right';
  ctx.fillText('255', paddingLeft - 15, paddingTop + 5);
  ctx.fillText('0', paddingLeft - 15, paddingTop + chartHeight / 2 + 5);
  ctx.fillText('-255', paddingLeft - 15, paddingTop + chartHeight + 5);
  
  const getX = (idx) => paddingLeft + (chartWidth * idx) / (samples.length - 1);
  const getMotorY = (pwm) => {
    const norm = pwm / 255.0; // [-1.0, 1.0]
    return paddingTop + chartHeight / 2 - (norm * (chartHeight / 2));
  };
  const getServoY = (angle) => {
    const center = 90;
    const diff = angle - center; // [-50, 50]
    const norm = diff / 50.0; // [-1.0, 1.0]
    return paddingTop + chartHeight / 2 - (norm * (chartHeight / 2));
  };
  
  // Draw Servo Line (Orange)
  ctx.beginPath();
  ctx.strokeStyle = '#f97316';
  ctx.lineWidth = 3;
  ctx.lineJoin = 'round';
  samples.forEach((s, idx) => {
    const x = getX(idx);
    const y = getServoY(s.servo_angle);
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
  
  // Draw Motor Line (Blue)
  ctx.beginPath();
  ctx.strokeStyle = '#3b82f6';
  ctx.lineWidth = 3;
  ctx.lineJoin = 'round';
  samples.forEach((s, idx) => {
    const x = getX(idx);
    const y = getMotorY(s.motor_pwm);
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function selectRecording(name) {
  if (!name) return;
  fetch('/api/recording_data?name=' + encodeURIComponent(name))
    .then(r => r.json())
    .then(res => {
      if (res.ok && res.data) {
        loadedRecordingSamples = res.data.samples || [];
        loadedRecordingName = res.data.name || '';
        drawTimeline(loadedRecordingSamples, loadedRecordingName);
        
        document.querySelectorAll('.recording-item').forEach(el => {
          const itemTitle = el.querySelector('.rec-name').textContent.trim();
          el.classList.toggle('selected', itemTitle === name);
        });
      }
    });
}

// ── Render Saved Recordings ──
function renderRecordings(saved, activeName, currentName) {
  const listEl = document.getElementById('recordingsList');
  if (!saved || saved.length === 0) {
    listEl.innerHTML = '<div class="no-recordings">No saved recordings found.</div>';
    return;
  }
  
  let html = '';
  saved.forEach(rec => {
    const isActive = rec.name === activeName;
    const isCurrent = rec.name === currentName;
    const isSelected = rec.name === loadedRecordingName;
    
    html += `
      <div class="recording-item ${isCurrent ? 'current' : ''} ${isActive ? 'active-parking' : ''} ${isSelected ? 'selected' : ''}" onclick="selectRecording('${rec.name}')">
        <div class="rec-meta">
          <div class="rec-name-row">
            <span class="rec-name">${escapeHtml(rec.name)}</span>
            <div style="display:flex; gap:4px;">
              ${isActive ? '<span class="star-badge">🅿 Active</span>' : ''}
              ${isCurrent ? '<span class="load-badge">Loaded</span>' : ''}
            </div>
          </div>
          <div class="rec-details">
            <span>📊 ${rec.sample_count} samples</span>
            <span>⏱ ${rec.duration_sec.toFixed(1)}s</span>
            <span class="rec-date">${rec.created_at}</span>
          </div>
        </div>
        <div class="rec-actions" onclick="event.stopPropagation();">
          <button class="action-btn load" onclick="loadRec('${rec.name}')">Load</button>
          <button class="action-btn play" onclick="playRec('${rec.name}')">Play</button>
          <button class="action-btn set-active" onclick="setActiveRec('${rec.name}')">Set Park</button>
          <button class="action-btn delete" onclick="deleteRec('${rec.name}')">🗑</button>
        </div>
      </div>
    `;
  });
  listEl.innerHTML = html;
}

// ── Main Data Update Loop ──
let lastRpState = 'IDLE';

function update() {
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      // Mode
      const mb = document.getElementById('modeBadge');
      mb.textContent = d.auto_mode ? 'AUTO' : 'MANUAL';
      mb.className = 'mode-pill ' + (d.auto_mode ? 'auto' : 'manual');

      // Odom
      document.getElementById('odomDist').textContent = (d.distance || 0).toFixed(2);
      document.getElementById('odomSpeed').textContent = (d.speed || 0).toFixed(3);
      document.getElementById('posX').textContent = (d.odom_x || 0).toFixed(2);
      document.getElementById('posY').textContent = (d.odom_y || 0).toFixed(2);
      document.getElementById('logYaw').textContent = (d.odom_yaw || 0).toFixed(3);

      // Steering
      const err = d.lane_error || 0;
      document.getElementById('steerErr').textContent = err > 0 ? '+' + err.toFixed(3) : err.toFixed(3);
      const sb = document.getElementById('steerValBlock');
      if (Math.abs(err) > 0.3) sb.className = 'data-val';
      else if (Math.abs(err) > 0.1) sb.className = 'data-val yellow';
      else sb.className = 'data-val green';

      // Odom status dot
      const dot = document.getElementById('odomDot');
      const odomAge = d.freshness_sec ? (d.freshness_sec.odom || d.freshness_sec.odom_sim || 999) : 999;
      if (odomAge < 2) { dot.className = 'status-dot live'; }
      else if (odomAge < 5) { dot.className = 'status-dot stale'; }
      else { dot.className = 'status-dot offline'; }

      // ── Record & Playback State ──
      const rpState = d.rp_state || 'IDLE';
      const bufSize = d.rp_buffer_size || 0;
      const pbIdx = d.rp_playback_index || 0;
      const curName = d.rp_recording_name || '';
      const actName = d.rp_active_parking || '';
      const savedList = d.rp_saved_recordings || [];

      // State badge
      const badge = document.getElementById('rpStateBadge');
      badge.textContent = rpState;
      badge.className = 'rp-state-badge ' + rpState.toLowerCase();

      // Buffer info
      document.getElementById('rpBufferSize').textContent = bufSize;
      const estDuration = (bufSize * 0.05).toFixed(1);
      document.getElementById('rpDuration').textContent = estDuration;

      // Button states
      const btnRec = document.getElementById('rpBtnRecord');
      const btnStop = document.getElementById('rpBtnStop');
      const btnPlay = document.getElementById('rpBtnPlay');
      const btnSave = document.getElementById('rpBtnSave');

      btnRec.classList.toggle('active', rpState === 'RECORDING');
      btnPlay.classList.toggle('active', rpState === 'PLAYBACK');

      btnRec.disabled = (rpState === 'PLAYBACK');
      btnPlay.disabled = (rpState === 'RECORDING' || bufSize === 0);
      btnStop.disabled = (rpState === 'IDLE');
      btnSave.disabled = (rpState !== 'IDLE' || bufSize === 0);

      // Progress bar
      const progressTrack = document.getElementById('rpProgress');
      const progressFill = document.getElementById('rpProgressFill');
      if (rpState === 'PLAYBACK' && bufSize > 0) {
        progressTrack.style.display = 'block';
        const pct = Math.min(100, (pbIdx / bufSize) * 100);
        progressFill.style.width = pct + '%';
      } else {
        progressTrack.style.display = 'none';
        progressFill.style.width = '0%';
      }

      // Render recordings list & Active Parking Badge
      document.getElementById('activeParkingBadge').textContent = actName ? actName : 'None';
      renderRecordings(savedList, actName, curName);

      // Draw timeline if we just recorded/loaded something new, or drawing the current running buffer
      if (rpState === 'RECORDING' || (rpState === 'PLAYBACK' && !loadedRecordingName)) {
        // Build a dummy/live timeline of the current recording buffer
        const dummySamples = [];
        for (let i = 0; i < bufSize; i++) {
          dummySamples.push({ motor_pwm: 0, servo_angle: 90 });
        }
        drawTimeline(dummySamples, rpState === 'RECORDING' ? 'Recording...' : 'Replaying...');
      } else if (curName && curName !== loadedRecordingName) {
        // Automatically select/draw the loaded recording
        selectRecording(curName);
      } else if (!curName && !loadedRecordingName && bufSize > 0) {
        // Loaded default/unsaved buffer
        const dummySamples = [];
        for (let i = 0; i < bufSize; i++) {
          dummySamples.push({ motor_pwm: 0, servo_angle: 90 });
        }
        drawTimeline(dummySamples, 'Unsaved Buffer');
      }

      // Auto-save dialog trigger when stopping recording
      if (lastRpState === 'RECORDING' && rpState === 'IDLE' && bufSize > 0) {
        showSaveModal();
      }

      lastRpState = rpState;
    })
    .catch(() => {});
}

update();
</script>
</body>
</html>"""

