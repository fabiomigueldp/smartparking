// Import required modules
const fs = require('fs');             // File system module for reading SSL certificate files
const https = require('https');       // HTTPS module to create a secure server
const WebSocket = require('ws');      // WebSocket module for handling WebSocket connections

// Configure SSL certificate paths
const options = {
  key: fs.readFileSync('/etc/letsencrypt/live/smartparking.torbware.space/privkey.pem'),      // Private key for SSL
  cert: fs.readFileSync('/etc/letsencrypt/live/smartparking.torbware.space/fullchain.pem')     // Certificate chain for SSL
};

// Create an HTTPS server on port 3241
const server = https.createServer(options, (req, res) => {
  res.writeHead(200, { 'Content-Type': 'text/plain' }); // Set response header to plain text
  res.end("Secure WebSocket Server is running on port 3241.\n"); // Send response message
});

// Initialize a WebSocket Server, specifying the HTTPS server and the path '/ws'
const wss = new WebSocket.Server({ server, path: "/ws" });

// Handle new client connections
wss.on('connection', (ws) => {
  console.log('New client connected via WSS (port 3241)'); // Log new connection

  // Listen for messages from the client (e.g., Python script or another client)
  ws.on('message', (data) => {
    try {
      const jsonData = JSON.parse(data); // Parse the incoming message as JSON

      // Broadcast the received message to all connected clients
      wss.clients.forEach(client => {
        // Check if the client is ready to receive messages
        if (client.readyState === WebSocket.OPEN) {
          client.send(JSON.stringify(jsonData)); // Send the JSON message
        }
      });
    } catch (err) {
      console.error('Error parsing message:', err); // Log parsing errors
    }
  });

  // Handle client disconnection
  ws.on('close', () => {
    console.log('Client disconnected'); // Log disconnection
  });
});

// Define the port number for the server to listen on
const port = 3241;

// Start the HTTPS server and listen on the specified port
server.listen(port, () => {
  console.log(`Secure WebSocket server listening on port ${port}`); // Log server start
});
