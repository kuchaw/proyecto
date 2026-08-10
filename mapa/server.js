const express = require("express");
const path = require("path");

const app = express();

app.use(express.json());
app.use(express.static(path.join(__dirname)));

let telemetryHistory = [];

app.get("/api/telemetry", (req, res) => {
  res.json(telemetryHistory);
});

app.post("/api/telemetry", (req, res) => {
  const data = req.body;
  const now = new Date();

  const entry = {
    packetType: data.packetType ?? null,
    version: data.version ?? null,
    counter: data.counter ?? null,
    mode: data.mode ?? null,
    lidar_status: data.lidar_status ?? null,
    mpu_status: data.mpu_status ?? null,
    mission_time_ms: data.time_ms ?? data.mission_time_ms ?? null,

    // GPS
    lat: data.lat ?? null,
    lon: data.lon ?? null,
    alt: data.alt ?? null,
    sat: data.sat ?? null,
    speed: data.speed ?? null,
    course: data.course ?? null,
    lidar_mm: data.lidar_mm ?? null,
    // Main environment fields used by current HTML
    temp: data.temp ?? null,
    pressure: data.pressure ?? null,
    humidity: data.humidity ?? null,

    // Extra sensors
    //temp_bme: data.temp_bme ?? null,
    gas: data.gas ?? null,

    // IMU
    roll: data.roll_deg10 != null ? data.roll_deg10 / 10 : null,
    pitch: data.pitch_deg10 != null ? data.pitch_deg10 / 10 : null,
    yaw: data.yaw_deg10 != null ? data.yaw_deg10 / 10 : null,
    // Time
    date: data.date || now.toISOString().split("T")[0],
    time: data.time || now.toTimeString().split(" ")[0]
  };

  telemetryHistory.push(entry);

  if (telemetryHistory.length > 100) {
    telemetryHistory.shift();
  }

  console.log("Stored:", entry);

  res.status(200).json({ ok: true });
});

app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "mapa.html"));
});

const PORT = process.env.PORT || 3000;

app.listen(PORT, () => {
  console.log("Running on port", PORT);
});