const express = require("express");
const path = require("path");

const app = express();

app.use(express.json());
app.use(express.static(path.join(__dirname)));


// ======================================================
// TELEMETRY STORAGE
// ======================================================

// 2000 muestras a 750 ms por muestra son aproximadamente
// 25 minutos de telemetría.
const MAX_HISTORY = 2000;

let telemetryHistory = [];


// ======================================================
// HELPERS
// ======================================================

function numberOrNull(value) {
  if (
    value === null ||
    value === undefined ||
    value === ""
  ) {
    return null;
  }

  const n = Number(value);

  return Number.isFinite(n)
    ? n
    : null;
}


// ======================================================
// GET TELEMETRY HISTORY
// ======================================================

app.get("/api/telemetry", (req, res) => {
  res.json(telemetryHistory);
});


// ======================================================
// GET LATEST TELEMETRY SAMPLE
// ======================================================

app.get("/api/latest", (req, res) => {

  if (telemetryHistory.length === 0) {
    return res.status(404).json({
      ok: false,
      message: "No telemetry received yet"
    });
  }

  res.json(
    telemetryHistory[
      telemetryHistory.length - 1
    ]
  );
});


// ======================================================
// POST TELEMETRY
// ======================================================

app.post("/api/telemetry", (req, res) => {

  const data = req.body;
  const now = new Date();


  const entry = {

    // ==================================================
    // Mission
    // ==================================================

    counter:
      numberOrNull(data.counter),

    time_ms:
      numberOrNull(data.time_ms),

    mode:
      numberOrNull(data.mode),


    // ==================================================
    // GPS
    // ==================================================

    lat:
      numberOrNull(data.lat),

    lon:
      numberOrNull(data.lon),

    alt:
      numberOrNull(data.alt),

    speed:
      numberOrNull(data.speed),

    sat:
      numberOrNull(data.sat),


    // ==================================================
    // BME680
    // ==================================================

    temp:
      numberOrNull(data.temp),

    humidity:
      numberOrNull(data.humidity),

    pressure:
      numberOrNull(data.pressure),

    gas_kohm:
      numberOrNull(data.gas_kohm),


    // ==================================================
    // MPU6050
    // Acceleration is received from the ground ESP32
    // already converted to m/s^2.
    // ==================================================

    accel_x:
      numberOrNull(data.accel_x),

    accel_y:
      numberOrNull(data.accel_y),

    accel_z:
      numberOrNull(data.accel_z),

    accel_total:
      numberOrNull(data.accel_total),


    // ==================================================
    // ESP-NOW link
    // ==================================================

    espnow_rssi:
      numberOrNull(data.espnow_rssi),


    // ==================================================
    // Server reception time
    // ==================================================

    date:
      now.toISOString().split("T")[0],

    time:
      now.toTimeString().split(" ")[0],

    received_at:
      now.toISOString()
  };


  // ====================================================
  // Basic packet validation
  // ====================================================

  if (entry.counter === null) {

    return res.status(400).json({
      ok: false,
      error: "Missing or invalid counter"
    });

  }


  // ====================================================
  // Store telemetry
  // ====================================================

  telemetryHistory.push(entry);


  // Keep history bounded in RAM.
  if (telemetryHistory.length > MAX_HISTORY) {
    telemetryHistory.shift();
  }


  // ====================================================
  // Server console output
  // ====================================================

  console.log(
    `Stored telemetry #${entry.counter} | ` +
    `mode=${entry.mode} | ` +
    `alt=${entry.alt} m | ` +
    `speed=${entry.speed} km/h | ` +
    `RSSI=${entry.espnow_rssi} dBm`
  );


  // ====================================================
  // Response to ground ESP32
  // ====================================================

  res.status(200).json({
    ok: true,
    counter: entry.counter,
    stored: telemetryHistory.length
  });

});


// ======================================================
// ROOT PAGE
// ======================================================

app.get("/", (req, res) => {

  res.sendFile(
    path.join(
      __dirname,
      "mapa.html"
    )
  );

});


// ======================================================
// SERVER START
// ======================================================

const PORT =
  process.env.PORT || 3000;


app.listen(PORT, () => {

  console.log(
    "======================================"
  );

  console.log(
    "Hermes telemetry server running"
  );

  console.log(
    "Port:",
    PORT
  );

  console.log(
    "Max history:",
    MAX_HISTORY,
    "samples"
  );

  console.log(
    "======================================"
  );

});