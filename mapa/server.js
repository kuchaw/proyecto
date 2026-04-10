const express = require("express");
const path = require("path");

const app = express();

app.use(express.json());
app.use(express.static(path.join(__dirname)));

let telemetryHistory = [];

// TEST ROUTE (very important)
app.get("/test", (req, res) => {
  res.send("OK");
});

// API
app.get("/api/telemetry", (req, res) => {
  res.json(telemetryHistory);
});

app.post("/api/telemetry", (req, res) => {
  console.log("🔥 BODY:", req.body);

  const now = new Date();

  const entry = {
    lat: req.body.lat,
    lon: req.body.lon,
    alt: req.body.alt,
    sat: req.body.sat,
    temp: req.body.temp,
    pressure: req.body.pressure,
    humidity: req.body.humidity,
    time: now.toISOString()
  };

  telemetryHistory.push(entry);

  console.log("🔥 STORED:", entry);

  res.sendStatus(200);
});

// ROOT
app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "mapa.html"));
});

// CRASH DEBUG
process.on("uncaughtException", (err) => {
  console.error("🔥 ERROR:", err);
});

const PORT = process.env.PORT || 3000;

app.listen(PORT, () => {
  console.log("🚀 Running on port", PORT);
});