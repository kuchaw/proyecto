const express = require("express");

const app = express();

app.use(express.json());
app.use(express.static(__dirname));

let telemetryHistory = [];

app.post("/api/telemetry", (req, res) => {
  const data = req.body;

  const now = new Date();

  const entry = {
    lat: data.lat,
    lon: data.lon,
    alt: data.alt,
    sat: data.sat,
    date: now.toISOString().split("T")[0],
    time: now.toTimeString().split(" ")[0]
  };

  telemetryHistory.push(entry);
  if (telemetryHistory.length > 100) {
    telemetryHistory.shift(); // remove oldest
  }

  console.log("Stored:", entry);

  res.sendStatus(200);
});

app.get("/api/telemetry", (req, res) => {
  res.json(telemetryHistory);
  res.send("new version running");
});

const PORT = process.env.PORT || 3000;

app.listen(PORT, "0.0.0.0", () => {
  console.log("Running on port", PORT);
});