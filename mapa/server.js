const express = require("express");

const app = express();

app.use(express.json());
app.use(express.static(__dirname));

let latestTelemetry = null;


app.post("/api/telemetry", (req, res) => {
  latestTelemetry = req.body;
  console.log("Received:", latestTelemetry);
  res.sendStatus(200);
});

app.get("/api/telemetry", (req, res) => {
  res.json(latestTelemetry || null);
});

const PORT = process.env.PORT || 3000;

app.listen(PORT, () => {
  console.log("Running on port jfjjyjd6jyj", PORT);
});