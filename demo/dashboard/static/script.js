// UAV Dashboard - Main JavaScript

// Global variables
let map;
let socket;
let droneMarkers = {};
let waypoints = [];
let waypointMarkers = [];
let selectedDrone = null;
let waypointMode = false;

// Initialize the dashboard
document.addEventListener('DOMContentLoaded', function() {
    initializeMap();
    initializeWebSocket();
    setupEventListeners();
    updateTime();
    setInterval(updateTime, 1000);
    
    // Debug info
    console.log("UAV Dashboard initialized");
});

// Initialize the Leaflet map
function initializeMap() {
    // Create map centered on a default location (will be updated with drone locations)
    const defaultLocation = [39.3290775819491, -76.62073373794556]; // Default to tower location
    map = L.map('map-container').setView(defaultLocation, 15);
    
    // Add the main tile layer (standard map)
    const standardLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        maxZoom: 21
    }).addTo(map);

    // Add satellite layer (not shown by default)
    const satelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
        maxZoom: 21
    });
    
    // Store layers for toggling
    const baseLayers = {
        "Standard": standardLayer,
        "Satellite": satelliteLayer
    };
    
    // Add layer control to map
    L.control.layers(baseLayers).addTo(map);
    
    // Handle map clicks for waypoints
    map.on('click', function(e) {
        if (waypointMode && selectedDrone) {
            addWaypoint(e.latlng);
        }
    });
    
    console.log("Map initialized");
}

// Initialize WebSocket connection
function initializeWebSocket() {
    try {
        // Connect to the server
        const wsProtocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
        const wsURL = `${wsProtocol}${window.location.host}`;
        
        console.log(`Connecting to WebSocket at ${wsURL}`);
        
        // For Socket.IO connection
        socket = io(wsURL);
        
        // Socket connection events
        socket.on('connect', function() {
            console.log("WebSocket connected");
            updateConnectionStatus(true);
            addNotification('Connected to server', 'success');
            
            // Request initial data
fetch('/api/data')
    .then(response => response.json())
    .then(data => {
                    console.log("Initial data received:", data);
                    if (data.drones) {
        updateDroneList(data.drones);
                    }
                })
                .catch(error => {
                    console.error('Error fetching initial data:', error);
                });
        });
        
        socket.on('disconnect', function() {
            console.log("WebSocket disconnected");
            updateConnectionStatus(false);
            addNotification('Disconnected from server', 'error');
        });
        
        // Handle data updates from server
        socket.on('data_update', function(data) {
            console.log("Data update received:", data);
            
            // Log all drone data for debugging
            if (data.drones) {
                Object.entries(data.drones).forEach(([id, drone]) => {
                    console.log(`Drone ${id} data:`, drone);
                });
                
                updateDroneList(data.drones);
            }
        });
        
        // Handle command acknowledgments
        socket.on('command_ack', function(data) {
            console.log("Command acknowledgment:", data);
            const type = data.success ? 'success' : 'error';
            addNotification(data.message, type);
        });
        
        // Handle waypoint creation feedback
        socket.on('waypoint_created', function(data) {
            console.log("Waypoint created:", data);
            if (data.drone_id === selectedDrone) {
                addNotification(`Waypoint set at ${data.target.lat.toFixed(6)}, ${data.target.lon.toFixed(6)}`, 'success');
            }
        });
        
        // Custom event handler for handling direct socket messages
        socket.on('message', function(data) {
            console.log("Direct message received:", data);
            try {
                // Try to parse as JSON
                const jsonData = (typeof data === 'string') ? JSON.parse(data) : data;
                
                if (jsonData.drone_id) {
                    // This appears to be drone data in JSON format
                    console.log("Parsed drone data from message:", jsonData);
                    
                    // Update the drone data 
                    if (jsonData.drone_id && drone_data[jsonData.drone_id]) {
                        drone_data[jsonData.drone_id] = {
                            ...drone_data[jsonData.drone_id],
                            ...jsonData,
                            status: drone_data[jsonData.drone_id].status || "CONNECTED"
                        };
                        
                        // Update the UI
                        updateDroneList(drone_data);
                    }
                }
            } catch (e) {
                console.log("Non-JSON message received:", data);
            }
        });
    } catch (error) {
        console.error("Error initializing WebSocket:", error);
        addNotification(`WebSocket error: ${error.message}`, 'error');
    }
}

// Set up all event listeners
function setupEventListeners() {
    // Command selection change
    const commandSelect = document.getElementById('command-select');
    if (commandSelect) {
        commandSelect.addEventListener('change', function() {
            console.log(`Command selected: ${this.value}`);
            // Hide all command fields
            document.querySelectorAll('.command-fields').forEach(el => {
                el.classList.remove('active');
            });
            
            // Show the selected command fields
            const selectedValue = this.value;
            if (selectedValue) {
                const fieldsToShow = document.getElementById(`${selectedValue}-fields`);
                if (fieldsToShow) {
                    fieldsToShow.classList.add('active');
                    console.log(`Showing fields for: ${selectedValue}`);
                    
                    // If waypoint is selected and we have a selected drone but no waypoints,
                    // calculate for an arbitrary point
                    if (selectedValue === 'waypoint' && selectedDrone && waypoints.length === 0) {
                        calculateArbitraryNedPoint();
                    }
                } else {
                    console.warn(`Fields not found for: ${selectedValue}`);
                }
            }
        });
    } else {
        console.error("Command select element not found");
    }
    
    // Command form submission
    const commandForm = document.getElementById('command-form');
    if (commandForm) {
        commandForm.addEventListener('submit', function(e) {
            e.preventDefault();
            if (selectedDrone) {
                sendDroneCommand();
            } else {
                addNotification('Please select a drone first', 'error');
            }
        });
    } else {
        console.error("Command form not found");
    }
    
    // Waypoint mode toggle
    const waypointToggle = document.getElementById('waypoint-mode-toggle');
    if (waypointToggle) {
        waypointToggle.addEventListener('click', function() {
            waypointMode = !waypointMode;
            this.textContent = waypointMode ? 'Exit Waypoint Mode' : 'Enter Waypoint Mode';
            
            const instructions = document.getElementById('map-instructions');
            if (instructions) {
                instructions.classList.toggle('hidden', !waypointMode);
            }
            
            // Change map cursor
            if (map) {
                map.getContainer().style.cursor = waypointMode ? 'crosshair' : '';
            }
            
            // If entering waypoint mode and we have a selected drone but no waypoints,
            // calculate for an arbitrary point
            if (waypointMode && selectedDrone && waypoints.length === 0) {
                calculateArbitraryNedPoint();
            }
            
            addNotification(waypointMode ? 'Click on map to add waypoints' : 'Waypoint mode disabled', 'info');
        });
    }
    
    // Clear waypoints button
    const clearWaypointsBtn = document.getElementById('clear-waypoints');
    if (clearWaypointsBtn) {
        clearWaypointsBtn.addEventListener('click', clearWaypoints);
    }
    
    // Clear notifications button
    const clearNotificationsBtn = document.getElementById('clear-notifications');
    if (clearNotificationsBtn) {
        clearNotificationsBtn.addEventListener('click', function() {
            document.getElementById('notifications-list').innerHTML = '';
        });
    }
    
    // Map centering button
    const centerMapBtn = document.getElementById('center-map');
    if (centerMapBtn) {
        centerMapBtn.addEventListener('click', function() {
            if (selectedDrone && droneMarkers[selectedDrone]) {
                map.setView(droneMarkers[selectedDrone].getLatLng(), map.getZoom());
            }
        });
    }
    
    // Toggle satellite view
    const toggleSatelliteBtn = document.getElementById('toggle-satellite');
    if (toggleSatelliteBtn) {
        toggleSatelliteBtn.addEventListener('click', function() {
            // This would normally toggle the satellite layer
            // Since we're using Leaflet's built-in layer control, this is handled there
            addNotification('Use the layer control in the top-right of the map to switch views', 'info');
        });
    }
    
    // Refresh drones button
    const refreshDronesBtn = document.getElementById('refresh-drones');
    if (refreshDronesBtn) {
        refreshDronesBtn.addEventListener('click', function() {
            console.log("Manually refreshing drone data");
            addNotification('Requesting drone data refresh...', 'info');
            
            // First try the REFRESH command via socket
            if (socket && socket.connected) {
                socket.emit('send_command', {
                    drone_id: 'all',
                    command: 'REFRESH'
                });
            }
            
            // Then request fresh data from server
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    console.log("Refresh data received:", data);
                    if (data.drones) {
                        updateDroneList(data.drones);
                        addNotification('Drone data refreshed', 'success');
                    }
                })
                .catch(error => {
                    console.error('Error fetching drone data:', error);
                    addNotification('Failed to refresh drone data', 'error');
                });
        });
    }
    
    console.log("Event listeners set up");
}

// Update the drone list display
function updateDroneList(drones) {
    const droneList = document.getElementById('drone-list');
    if (!droneList) {
        console.error("Drone list element not found");
        return;
    }
    
    console.log("Updating drone list with data:", drones);
    
    // Clear loading message and existing drone displays
    droneList.innerHTML = '';
    
    // Also clear map markers for disconnected drones
    const connectedDroneIds = Object.keys(drones);
    Object.keys(droneMarkers).forEach(id => {
        if (!connectedDroneIds.includes(id)) {
            // Remove marker for disconnected drone
            if (droneMarkers[id]) {
                map.removeLayer(droneMarkers[id]);
                delete droneMarkers[id];
                console.log(`Removed marker for disconnected drone ${id}`);
            }
        }
    });
    
    // Check if there are any drones
    if (Object.keys(drones).length === 0) {
        droneList.innerHTML = '<div class="loading-placeholder">No drones available</div>';
        console.warn("No drones available");
        return;
    }
    
    // Include ALL drones in the list, even those with "NOT INITIALIZED" status
    // if they have coordinates or other data fields
    const activeDrones = {};
    Object.entries(drones).forEach(([id, drone]) => {
        // Consider a drone active if:
        // 1. It has a status that's not "NOT INITIALIZED", OR
        // 2. It's "NOT INITIALIZED" but has coordinates, altitude, velocity, or heading
        if (drone.status !== "NOT INITIALIZED" || 
            drone.lat !== undefined || 
            drone.lon !== undefined || 
            drone.alt !== undefined || 
            drone.vel !== undefined || 
            drone.hdg !== undefined) {
            
            // Add the drone to active drones
            activeDrones[id] = drone;
            console.log(`Drone ${id} status: ${drone.status}, has coordinates: ${drone.lat !== undefined && drone.lon !== undefined}`);
        } else {
            // Remove marker for inactive drone
            if (droneMarkers[id]) {
                map.removeLayer(droneMarkers[id]);
                delete droneMarkers[id];
                console.log(`Removed marker for inactive drone ${id}`);
            }
        }
    });
    
    // Check if there are any active drones
    if (Object.keys(activeDrones).length === 0) {
        droneList.innerHTML = '<div class="loading-placeholder">No active drones available</div>';
        console.warn("No active drones available");
        return;
    }
    
    // Create a drone list item for each active drone
    Object.entries(activeDrones).forEach(([id, drone]) => {
        console.log(`Processing drone ${id}:`, drone);
        
        // Create and append the drone element to the list
        const droneEl = createDroneElement(id, drone);
        droneList.appendChild(droneEl);
        
        // Update or create map marker for this drone if it has coordinates (even if they're 0,0)
        if (drone.lat !== undefined && drone.lon !== undefined) {
            updateDroneMarker(id, drone);
        } else {
            console.log(`Drone ${id} has no position data. Status: ${drone.status}`);
        }
    });
    
    // If the selected drone is no longer in the list, deselect it
    if (selectedDrone && !activeDrones[selectedDrone]) {
        selectedDrone = null;
        document.getElementById('command-form').classList.add('disabled');
    }
}

// Create a drone element for the list
function createDroneElement(id, drone) {
    const droneEl = document.createElement('div');
    droneEl.className = `drone-item${selectedDrone === id ? ' selected' : ''}`;
    droneEl.setAttribute('data-id', id);
    
    // Determine connection status - if a drone has data but status is "NOT INITIALIZED",
    // show it as connected anyway
    const hasData = drone.lat !== undefined || 
                    drone.lon !== undefined || 
                    drone.alt !== undefined || 
                    drone.vel !== undefined || 
                    drone.hdg !== undefined;
                    
    const isConnected = drone.status !== 'NOT INITIALIZED' || hasData;
    
    // Use actual battery level if provided, otherwise show unknown
    const batteryLevel = drone.battery || 'Unknown';
    let batteryClass = '';
    
    // Only apply battery classes if we have numerical battery data
    if (typeof batteryLevel === 'number') {
        if (batteryLevel < 30) batteryClass = 'battery-low';
        else if (batteryLevel < 70) batteryClass = 'battery-medium';
        else batteryClass = 'battery-high';
    }
    
    // Battery display differs based on whether we have battery data
    const batteryDisplay = typeof batteryLevel === 'number' ? 
        `<div class="battery-indicator">
            <div class="battery-level">
                <div class="battery-fill ${batteryClass}" style="width: ${batteryLevel}%"></div>
            </div>
            <span class="info-value">${batteryLevel}%</span>
        </div>` : 
        `<span class="info-value">Unknown</span>`;
    
    // Check if position data is available - treat lat/lon of 0.0 as valid
    const hasPosition = drone.lat !== undefined && drone.lon !== undefined;
    
    // Display status - if it's "NOT INITIALIZED" but has data, show "CONNECTED" instead
    const displayStatus = (drone.status === 'NOT INITIALIZED' && hasData) ? 'CONNECTED' : (drone.status || 'Unknown');
    
    droneEl.innerHTML = `
        <div class="drone-header">
            <div class="drone-name">Drone ${id}</div>
            <div class="drone-status ${isConnected ? 'connected' : 'disconnected'}">
                ${isConnected ? 'Connected' : 'Disconnected'}
            </div>
        </div>
        <div class="drone-info">
            <div class="drone-info-item">
                <span class="info-label">Status</span>
                <span class="info-value">${displayStatus}</span>
            </div>
            <div class="drone-info-item">
                <span class="info-label">Battery</span>
                ${batteryDisplay}
            </div>
            ${hasPosition ? `
            <div class="drone-info-item">
                <span class="info-label">Position</span>
                <span class="info-value">${Number(drone.lat).toFixed(6)}, ${Number(drone.lon).toFixed(6)}</span>
            </div>` : `
            <div class="drone-info-item">
                <span class="info-label">Position</span>
                <span class="info-value">No GPS data</span>
            </div>`
            }
            ${drone.alt !== undefined ? `
            <div class="drone-info-item">
                <span class="info-label">Altitude</span>
                <span class="info-value">${Number(drone.alt).toFixed(1)} m</span>
            </div>
            ` : ''}
            ${drone.vel !== undefined ? `
            <div class="drone-info-item">
                <span class="info-label">Speed</span>
                <span class="info-value">${Number(drone.vel).toFixed(1)} m/s</span>
            </div>
            ` : ''}
        </div>
    `;
    
    // Add click event to select this drone
    droneEl.addEventListener('click', function() {
        selectDrone(id);
    });
    
    return droneEl;
}

// Update or create a drone marker on the map
function updateDroneMarker(droneId, drone) {
    if (!map) {
        console.error("Map not initialized");
        return;
    }
    
    // Accept lat/lon of 0.0 as valid coordinates
    if (!drone || drone.lat === undefined || drone.lon === undefined) {
        console.warn(`Drone ${droneId} has no position data:`, drone);
        return;
    }
    
    const position = [drone.lat, drone.lon];
    console.log(`Updating marker for drone ${droneId} at position:`, position);
    
    // Create a custom icon for the drone
    const droneIcon = L.divIcon({
        className: 'drone-marker',
        html: `<div class="drone-heading-caret" style="transform: rotate(${drone.hdg ? drone.hdg : 0}deg);">^</div><i class=\"fas fa-drone\"></i>`,
        iconSize: [16, 24], // 16x16 for drone, caret above
        iconAnchor: [8, 16] // anchor at bottom of drone icon
    });
    
    // Update existing marker or create a new one
    if (droneMarkers[droneId]) {
        droneMarkers[droneId].setLatLng(position);
    } else {
        droneMarkers[droneId] = L.marker(position, { icon: droneIcon }).addTo(map);
        console.log(`Created new marker for drone ${droneId}`);
        
        // Add popup with drone info
        droneMarkers[droneId].bindPopup(`
            <strong>Drone ${droneId}</strong><br>
            Status: ${drone.status || 'Unknown'}<br>
            Altitude: ${drone.alt !== undefined ? Number(drone.alt).toFixed(1) + ' m' : 'Unknown'}<br>
            Speed: ${drone.vel !== undefined ? Number(drone.vel).toFixed(1) + ' m/s' : 'Unknown'}
        `);
        
        // Add click event to select this drone
        droneMarkers[droneId].on('click', function() {
            selectDrone(droneId);
        });
    }
    
    // If this is the selected drone, update the marker style
    if (droneId === selectedDrone) {
        if (droneMarkers[droneId].getElement()) {
            droneMarkers[droneId].getElement().classList.add('selected');
        }
    } else {
        if (droneMarkers[droneId].getElement()) {
            droneMarkers[droneId].getElement().classList.remove('selected');
        }
    }
    
    // Remove any previous vector code for heading
    if (window.droneVectors && window.droneVectors[droneId]) {
        map.removeLayer(window.droneVectors[droneId]);
        delete window.droneVectors[droneId];
    }
    
    // Add CSS for the caret in a <style> tag if not already present
    if (!document.getElementById('drone-heading-caret-style')) {
        const style = document.createElement('style');
        style.id = 'drone-heading-caret-style';
        style.innerHTML = `.drone-heading-caret { display: block; color: #e74c3c; font-size: 14px; font-weight: bold; line-height: 12px; text-align: center; margin-bottom: -2px; }`;
        document.head.appendChild(style);
    }
}

// Select a drone
function selectDrone(droneId) {
    console.log(`Selecting drone ${droneId}`);
    
    // Deselect previously selected drone
    if (selectedDrone) {
        const prevElement = document.querySelector(`.drone-item[data-id="${selectedDrone}"]`);
        if (prevElement) {
            prevElement.classList.remove('selected');
        }
        if (droneMarkers[selectedDrone] && droneMarkers[selectedDrone].getElement()) {
            droneMarkers[selectedDrone].getElement().classList.remove('selected');
        }
    }
    
    // Select new drone
    selectedDrone = droneId;
    const element = document.querySelector(`.drone-item[data-id="${droneId}"]`);
    if (element) {
        element.classList.add('selected');
    }
    if (droneMarkers[droneId] && droneMarkers[droneId].getElement()) {
        droneMarkers[droneId].getElement().classList.add('selected');
    }
    
    // Enable command form
    const commandForm = document.getElementById('command-form');
    if (commandForm) {
        commandForm.classList.remove('disabled');
        // Always enable NED fields
        document.getElementById('waypoint-x').removeAttribute('readonly');
        document.getElementById('waypoint-x').removeAttribute('disabled');
        document.getElementById('waypoint-y').removeAttribute('readonly');
        document.getElementById('waypoint-y').removeAttribute('disabled');
        document.getElementById('waypoint-z').removeAttribute('readonly');
        document.getElementById('waypoint-z').removeAttribute('disabled');
    } else {
        console.error("Command form not found when selecting drone");
    }
    
    // Center map on selected drone if it has a marker (which means it has GPS coordinates)
    if (droneMarkers[droneId]) {
        map.setView(droneMarkers[droneId].getLatLng(), map.getZoom());
        
        // If we have waypoints, update NED coordinates
        if (waypoints.length > 0) {
            updateNedCoordinates();
        } else {
            // Calculate for an arbitrary point 10 meters north and 5 meters east of the drone
            calculateArbitraryNedPoint();
        }
    } else {
        // The drone doesn't have valid GPS coordinates
        console.log(`Drone ${droneId} doesn't have a valid position for map centering`);
        // Still attempt to show example waypoint coordinates
        calculateArbitraryNedPoint();
    }
    
    addNotification(`Drone ${droneId} selected`, 'info');
}

// Calculate NED coordinates for an arbitrary point relative to the drone
function calculateArbitraryNedPoint() {
    if (!selectedDrone) {
        return;
    }
    
    // Define arbitrary offsets (10m North, 5m East)
    const northOffset = 10; // meters
    const eastOffset = 5;   // meters
    const downOffset = 0;   // meters (no change in altitude)
    
    // Fill in the NED coordinate fields with the arbitrary values
    // These values will be used if drone doesn't have GPS coordinates
    document.getElementById('waypoint-x').value = northOffset.toFixed(1);
    document.getElementById('waypoint-y').value = eastOffset.toFixed(1);
    document.getElementById('waypoint-z').value = downOffset.toFixed(1);
    
    // Check if the drone has valid GPS coordinates and a marker on the map
    if (droneMarkers[selectedDrone]) {
        addNotification(`Example NED coordinates shown (${northOffset}m N, ${eastOffset}m E)`, 'info');
    } else {
        addNotification(`Drone has no GPS position. Using default NED coordinates (${northOffset}m N, ${eastOffset}m E)`, 'warning');
    }
}

// Add a waypoint to the map
function addWaypoint(latlng) {
    // Clear existing waypoints first (only one waypoint at a time)
    clearWaypoints();
    
    // Add the new waypoint
    waypoints.push(latlng);
    
    // Create a marker for the waypoint
    const waypointIcon = L.divIcon({
        className: 'waypoint-marker',
        html: `<div>W</div>`,
        iconSize: [20, 20],
        iconAnchor: [10, 10]
    });
    
    const marker = L.marker(latlng, {
        icon: waypointIcon,
        draggable: true
    }).addTo(map);
    
    // Update waypoint position when marker is dragged
    marker.on('dragend', function() {
        waypoints[0] = marker.getLatLng();
        // If a drone is selected, update the NED coordinates from backend
        if (selectedDrone && droneMarkers[selectedDrone]) {
            updateNedCoordinatesFromBackend();
        }
    });
    
    waypointMarkers.push(marker);
    
    // If a drone is selected, calculate and display NED coordinates from backend
    if (selectedDrone && droneMarkers[selectedDrone]) {
        updateNedCoordinatesFromBackend();
    }
    
    addNotification(`Waypoint added`, 'success');
}

// Calculate and update NED coordinates using the backend
function updateNedCoordinatesFromBackend() {
    if (!selectedDrone || !droneMarkers[selectedDrone] || waypoints.length === 0) {
        return;
    }
    const waypointPos = waypoints[0];
    fetch('/api/waypoint', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            drone_id: selectedDrone,
            target_lat: waypointPos.lat,
            target_lon: waypointPos.lng
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success && data.waypoint) {
            // Only update if not focused
            if (document.activeElement !== document.getElementById('waypoint-x')) {
                document.getElementById('waypoint-x').value = data.waypoint.x;
            }
            if (document.activeElement !== document.getElementById('waypoint-y')) {
                document.getElementById('waypoint-y').value = data.waypoint.y;
            }
            if (document.activeElement !== document.getElementById('waypoint-z')) {
                document.getElementById('waypoint-z').value = data.waypoint.z;
            }
        } else {
            addNotification('Failed to calculate NED coordinates', 'error');
        }
    })
    .catch(() => {
        addNotification('Error contacting backend for NED calculation', 'error');
    });
}

// Clear all waypoints
function clearWaypoints() {
    // Remove all waypoint markers from the map
    waypointMarkers.forEach(marker => {
        map.removeLayer(marker);
    });
    
    // Remove all polylines
    map.eachLayer(function(layer) {
        if (layer instanceof L.Polyline && !(layer instanceof L.Polygon)) {
            map.removeLayer(layer);
        }
    });
    
    // Clear the arrays
    waypoints = [];
    waypointMarkers = [];
    
    // Clear input fields
    const xInput = document.getElementById('waypoint-x');
    const yInput = document.getElementById('waypoint-y');
    const zInput = document.getElementById('waypoint-z');
    if (xInput) xInput.value = '0';
    if (yInput) yInput.value = '0';
    if (zInput) zInput.value = '0';
    
    addNotification('Waypoints cleared', 'info');
}

// Send a command to the selected drone
function sendDroneCommand() {
    if (!selectedDrone) {
        addNotification('Cannot send command - no drone selected', 'error');
        return;
    }
    
    if (!socket || socket.disconnected) {
        addNotification('Cannot send command - not connected to server', 'error');
        return;
    }
    
    const commandSelect = document.getElementById('command-select');
    if (!commandSelect) {
        console.error("Command select element not found");
        return;
    }
    
    const commandType = commandSelect.value;
    if (!commandType) {
        addNotification('Please select a command', 'error');
        return;
    }
    
    console.log(`Sending command ${commandType} to drone ${selectedDrone}`);
    
    let commandData = {
        drone_id: selectedDrone,
        command: commandType
    };
    
    // Add command-specific parameters
    switch (commandType) {
        case 'takeoff':
            const altitude = document.getElementById('takeoff-altitude').value;
            commandData.command = `TAKEOFF ${altitude}`;
            break;
            
        case 'land':
            commandData.command = 'LAND';
            break;
            
        case 'waypoint':
            // Check if we're in map-click waypoint mode with waypoints set
            if (waypointMode && waypoints.length > 0) {
                // Use map-selected waypoints
                const wp = waypoints[0]; // Get first waypoint
                
                // Emit a map click event with the waypoint data (no speed parameter)
                socket.emit('map_click', {
                    drone_id: selectedDrone,
                    lat: wp.lat,
                    lon: wp.lng
                });
                
                console.log("Sent map-click waypoint command:", {
                    drone_id: selectedDrone,
                    lat: wp.lat,
                    lon: wp.lng
                });
                
                // Clear waypoints after sending
                clearWaypoints();
                // Prevent sending a duplicate generic command
                return;
            } else {
                // Use NED coordinates shown in the input fields
                const x = document.getElementById('waypoint-x').value;
                const y = document.getElementById('waypoint-y').value;
                const z = document.getElementById('waypoint-z').value;
                
                // Validate input
                if (x === '' || y === '' || z === '') {
                    addNotification('Please enter valid NED coordinates', 'error');
                    return;
                }
                
                // Send waypoint command directly with NED coordinates
                commandData.command = `WAYPOINT ${x} ${y} ${z}`;
                
                console.log("Sent NED waypoint command:", {
                    drone_id: selectedDrone,
                    command: commandData.command
                });
            }
            break;
        case 'precision_land':
            commandData.command = 'PRECISION_LAND';
            break;
        case 'standby':
            commandData.command = 'STANDBY';
            break;
    }
    
    // Don't send duplicate command if we already sent via map_click
    if (commandType === 'waypoint' && waypointMode && waypoints.length > 0) {
        return;
    }
    
    // Send the command
    console.log("Sending command data:", commandData);
    socket.emit('send_command', commandData);
    addNotification(`Sending command: ${commandData.command}`, 'info');
}

// Add a notification to the notifications panel
function addNotification(message, type = 'info') {
    console.log(`Notification (${type}): ${message}`);
    
    const notificationsList = document.getElementById('notifications-list');
    if (!notificationsList) {
        console.error("Notifications list element not found");
        return;
    }
    
    // Create notification element
    const notification = document.createElement('div');
    notification.className = `notification-item ${type}`;
    
    // Get current time
    const now = new Date();
    const timeString = now.toLocaleTimeString();
    
    // Set notification content
    notification.innerHTML = `
        <div class="notification-header">
            <span class="notification-title">${getNotificationTitle(type)}</span>
            <span class="notification-time">${timeString}</span>
        </div>
        <div class="notification-content">${message}</div>
    `;
    
    // Add to the beginning of the list
    notificationsList.insertBefore(notification, notificationsList.firstChild);
    
    // Remove oldest notifications if there are too many
    while (notificationsList.children.length > 10) {
        notificationsList.removeChild(notificationsList.lastChild);
    }
    
    // Also show a toast notification
    try {
        Toastify({
            text: message,
            duration: 3000,
            gravity: "top",
            position: "right",
            className: `toast-${type}`,
            stopOnFocus: true
        }).showToast();
    } catch (error) {
        console.error("Error showing toast notification:", error);
    }
}

// Get title for notification based on type
function getNotificationTitle(type) {
    switch (type) {
        case 'success': return 'Success';
        case 'error': return 'Error';
        case 'warning': return 'Warning';
        default: return 'Information';
    }
}

// Update connection status in the UI
function updateConnectionStatus(connected) {
    const statusElement = document.getElementById('connection-status');
    if (statusElement) {
        statusElement.textContent = connected ? 'Connected' : 'Disconnected';
        statusElement.className = `status-value ${connected ? 'connected' : 'disconnected'}`;
    } else {
        console.error("Connection status element not found");
    }
    
    // Update system status
    const systemStatus = document.getElementById('system-status');
    if (systemStatus) {
        systemStatus.textContent = connected ? 'Online' : 'Offline';
        systemStatus.className = `status-value ${connected ? 'connected' : 'disconnected'}`;
    } else {
        console.error("System status element not found");
    }
}

// Update the time display
function updateTime() {
    const timeElement = document.getElementById('current-time');
    if (timeElement) {
        const now = new Date();
        timeElement.textContent = now.toLocaleTimeString();
    }
}
