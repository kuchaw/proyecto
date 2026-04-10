const express = require("express");

const app = express();

app.use(express.json());
app.use(express.static(__dirname));

let telemetryHistory = [];

app.post("/api/telemetry", (req, res) => {
  const data = req.body;

  const now = new Date();

  const entry = {

    lat: -34.6 + (Math.random() - 0.5) * 0.1,
    lon: -58.4 + (Math.random() - 0.5) * 0.1,
    alt: 100 + Math.random() * 50,
    sat: Math.floor(5 + Math.random() * 3),
    temp: 20 + Math.random() * 5,
    pressure: 1000 + Math.random() * 10,
    humidity: 50 + Math.random() * 20,
    date: now.toISOString().split("T")[0],
    time: now.toTimeString().split(" ")[0]
  };

  telemetryHistory.push(entry);
  if (telemetryHistory.length > 100) {
    telemetryHistory.shift(); // remove oldest
  }

  console.log("StoredFKñvskdncpaoivéiw:", entry);

  res.sendStatus(200);
});

app.get("/api/telemetry", (req, res) => {
  res.json(telemetryHistory);
});

const PORT = process.env.PORT || 3000;

app.get("/", (req, res) => {
  res.sendFile(__dirname + "/mapa.html");
});

app.listen(PORT, "0.0.0.0", () => {
  console.log("Running on port", PORT);
});