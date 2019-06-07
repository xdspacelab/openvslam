let express = require("express");
let http_server = require("http").Server(express());
let io_server = require("socket.io")(http_server);

let app = express();
let http_publisher = require("http").Server(app);
let io_publisher = require("socket.io")(http_publisher);

// setting express
app.set("views", __dirname + "/views");
app.set("view engine", "ejs");
app.use(express.static(__dirname + "/public"));

// render browser
app.get("/", function (req, res) {
  res.render("index.ejs");
});

io_server.on("connection", function (socket) {
  console.log(`Connected - ID: ${socket.id}`);

  socket.on("map_publish", function (msg) {
    io_publisher.emit("map_publish", msg);
  });

  socket.on("frame_publish", function (msg) {
    io_publisher.emit("frame_publish", { image: true, buffer: msg });
  });

  socket.on("disconnect", function () {
    console.log(`Disconnected - ID: ${socket.id}`);
  });
});

io_publisher.on("connection", function (socket) {
  socket.on("signal", function (msg) {
    io_server.emit("signal", msg);
  });
});

http_server.listen(3000, function () {
  console.log("WebSocket: listening on *:3000");
});

http_publisher.listen(3001, function () {
  console.log("HTTP server: listening on *:3001")
});
