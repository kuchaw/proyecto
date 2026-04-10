const express = require("express");
const path = require("path");

const app = express();

app.use(express.json());
app.use(express.static(path.join(__dirname)));

let telemetryHistory = [];

//
// 🔥 SIMULATION (this feeds your flashcards)
//
setInterval(() => {
  const now = new Date();

  const entry = {
    lat: -34.6 + (Math.random() - 0.5) * 0.01,
    lon: -58.4 + (Math.random() - 0.5) * 0.01,
    alt: 100 + Math.random() * 20,
    sat: Math.floor(5 + Math.random() * 3),
    temp: 20 + Math.random() * 5,
    pressure: 1000 + Math.random() * 10,
    humidity: 50 + Math.random() * 20,
    time: now.toTimeString().split(" ")[0]
  };

  telemetryHistory.push(entry);

  if (telemetryHistory.length > 100) {
    telemetryHistory.shift();
  }

  console.log("📡 Simulated:", entry);
}, 3000);

//
// API (your frontend uses this)
//
app.get("/api/telemetry", (req, res) => {
  res.json(telemetryHistory);
});

//
// POST (future real data)
//
app.post("/api/telemetry", (req, res) => {
  const entry = {
    ...req.body,
    time: new Date().toTimeString().split(" ")[0]
  };

  telemetryHistory.push(entry);

  res.sendStatus(200);
});

//
// ROOT
//
app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "mapa.html"));
});

const PORT = process.env.PORT || 3000;

app.listen(PORT, () => {
  console.log("Running on port", PORT);
});