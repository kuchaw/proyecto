const express = require("express");
const path = require("path");

const app = express();

app.use(express.json());
app.use(express.static(path.join(__dirname)));

// Avoid cached API responses while the dashboard is running live.
app.use((req, res, next) => {
  if (req.path.startsWith("/api/")) {
    res.set("Cache-Control", "no-store");
  }
  next();
});


// ======================================================
// TELEMETRY STORAGE
// ======================================================

// 2000 muestras a 750 ms por muestra son aproximadamente
// 25 minutos de telemetría.
const MAX_HISTORY = 2000;

let telemetryHistory = [];

// Long-poll clients waiting for the next telemetry packet.
// This lets the dashboard update as soon as a packet arrives instead of
// waiting for the next fixed polling interval.
const LATEST_WAIT_MS = 3000;
const MAX_PENDING_LATEST_CLIENTS = 20;
let pendingLatestClients = [];


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

function getLatestTelemetry() {
  if (telemetryHistory.length === 0) {
    return null;
  }

  return telemetryHistory[telemetryHistory.length - 1];
}

function removePendingLatestClient(res) {
  pendingLatestClients = pendingLatestClients.filter(
    client => client.res !== res
  );
}

function waitForNextTelemetry(res) {
  if (pendingLatestClients.length >= MAX_PENDING_LATEST_CLIENTS) {
    return res.status(503).json({
      ok: false,
      error: "Too many pending latest requests"
    });
  }

  const timeout = setTimeout(() => {
    removePendingLatestClient(res);

    if (!res.headersSent) {
      // No new packet arrived during the wait window.
      // The frontend keeps showing the last known values.
      res.status(204).end();
    }
  }, LATEST_WAIT_MS);

  pendingLatestClients.push({ res, timeout });

  res.on("close", () => {
    clearTimeout(timeout);
    removePendingLatestClient(res);
  });
}

function notifyLatestClients(entry) {
  const clients = pendingLatestClients;
  pendingLatestClients = [];

  for (const client of clients) {
    clearTimeout(client.timeout);

    if (!client.res.headersSent) {
      client.res.json(entry);
    }
  }
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

  const latest = getLatestTelemetry();

  if (!latest) {
    return res.status(404).json({
      ok: false,
      message: "No telemetry received yet"
    });
  }

  // Optional long-poll mode:
  // /api/latest?after=123 means "only answer with a new packet
  // if the latest counter is different from 123".
  const after = numberOrNull(req.query.after);

  if (after !== null && latest.counter === after) {
    return waitForNextTelemetry(res);
  }

  res.json(latest);
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

  // Wake any dashboard request waiting for the next packet.
  notifyLatestClients(entry);

  console.log(
    `Stored telemetry #${entry.counter} | ` +
    `mode=${entry.mode} | ` +
    `alt=${entry.alt} m | ` +
    `speed=${entry.speed} m/s | ` +
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