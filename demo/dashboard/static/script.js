// Initialize the map
const map = L.map('map').setView([-35.3632, 149.1652], 17);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
}).addTo(map);

// Store markers for drones and towers
const droneMarkers = {};
const towerMarkers = {};

// Connect to the WebSocket server
const socket = io();

// Initial data load
fetch('/api/data')
    .then(response => response.json())
    .then(data => {
        updateMap(data);
        updateDroneList(data.drones);
        updateDroneSelect(data.drones);
    });

// Listen for real-time updates
socket.on('data_update', (data) => {
    updateMap(data);
    updateDroneList(data.drones);
    updateDroneSelect(data.drones);
});

// Function to update the map with new data
function updateMap(data) {
    // Update drone markers
    for (const [id, drone] of Object.entries(data.drones)) {
        const position = [drone.lat, drone.lon];
        
        if (id in droneMarkers) {
            droneMarkers[id].setLatLng(position);
        } else {
            const droneIcon = L.divIcon({
                html: `<div class="drone-icon">🚁</div>`,
                className: 'drone-marker',
                iconSize: [30, 30],
                iconAnchor: [15, 15],  // Center the icon
                popupAnchor: [0, -15],  // Position popup above the icon
                className: 'drone-marker-fixed'  // New class for fixed-size icons
            });
            
            droneMarkers[id] = L.marker(position, {icon: droneIcon})
                .addTo(map)
                .bindPopup(`Drone ${id}<br>Status: ${drone.status}<br>Alt: ${drone.alt}m`);
        }
        
        // Update popup content
        droneMarkers[id].setPopupContent(`Drone ${id}<br>Status: ${drone.status}<br>Alt: ${drone.alt}m`);
    }
    
    // Update tower markers
    for (const [id, tower] of Object.entries(data.towers)) {
        const position = [tower.lat, tower.lon];
        
        if (id in towerMarkers) {
            towerMarkers[id].setLatLng(position);
        } else {
            const towerIcon = L.divIcon({
                html: `<div class="tower-icon">🗼</div>`,
                className: 'tower-marker',
                iconSize: [30, 30],
                iconAnchor: [15, 15],  // Center the icon
                popupAnchor: [0, -15],  // Position popup above the icon
                className: 'tower-marker-fixed'  // New class for fixed-size icons
            });
            
            towerMarkers[id] = L.marker(position, {icon: towerIcon})
                .addTo(map)
                .bindPopup(`Tower ${id}`);
        }
    }
}

// Function to update the drone list
function updateDroneList(drones) {
    const droneList = document.getElementById('drone-list');
    droneList.innerHTML = '';
    
    for (const [id, drone] of Object.entries(drones)) {
        const droneItem = document.createElement('div');
        droneItem.className = 'drone-item';
        droneItem.setAttribute('data-drone-id', id);
        droneItem.innerHTML = `
            <h3>Drone ${id}</h3>
            <p>Position: ${drone.lat.toFixed(6)}, ${drone.lon.toFixed(6)}</p>
            <p>Altitude: ${drone.alt.toFixed(1)}m</p>
            <p>Status: <span class="active-command">${drone.status}</span></p>
        `;
        droneList.appendChild(droneItem);
    }
}

// Function to update the drone select dropdown
function updateDroneSelect(drones) {
    const droneSelect = document.getElementById('drone-select');
    const currentValue = droneSelect.value;
    
    // Save current options to check against new ones
    const currentOptions = Array.from(droneSelect.options)
        .filter(option => option.value !== '')
        .map(option => option.value);
    
    // Add new drones
    for (const id in drones) {
        if (!currentOptions.includes(id)) {
            const option = document.createElement('option');
            option.value = id;
            option.textContent = `Drone ${id}`;
            droneSelect.appendChild(option);
        }
    }
    
    // Remove drones that no longer exist
    for (let i = droneSelect.options.length - 1; i >= 0; i--) {
        const option = droneSelect.options[i];
        if (option.value !== '' && !(option.value in drones)) {
            droneSelect.remove(i);
        }
    }
    
    // Restore selected value if it still exists
    if (currentValue in drones) {
        droneSelect.value = currentValue;
    }
}

// Handle command select change
document.getElementById('command-select').addEventListener('change', function() {
    const waypointInputs = document.getElementById('waypoint-inputs');
    const takeoffInputs = document.getElementById('takeoff-inputs');
    
    // Hide all parameter inputs first
    waypointInputs.style.display = 'none';
    takeoffInputs.style.display = 'none';
    
    // Show relevant inputs based on command
    if (this.value === 'WAYPOINT') {
        waypointInputs.style.display = 'block';
    } else if (this.value === 'TAKEOFF') {
        takeoffInputs.style.display = 'block';
    }
});

// Handle send command button click
document.getElementById('send-command').addEventListener('click', function() {
    const droneId = document.getElementById('drone-select').value;
    const commandType = document.getElementById('command-select').value;
    
    if (!droneId || !commandType) {
        alert('Please select a drone and command');
        return;
    }
    
    let command = commandType;
    
    // Add command parameters if needed
    if (commandType === 'WAYPOINT') {
        const x = document.getElementById('wp-x').value;
        const y = document.getElementById('wp-y').value;
        const z = document.getElementById('wp-z').value;
        
        if (!x || !y || !z) {
            alert('Please provide all waypoint coordinates');
            return;
        }
        
        command = `WAYPOINT ${x} ${y} ${z}`;
    } else if (commandType === 'TAKEOFF') {
        const alt = document.getElementById('takeoff-alt').value;
        
        if (!alt) {
            alert('Please provide takeoff altitude');
            return;
        }
        
        command = `TAKEOFF ${alt}`;
    }
    
    // Update the drone's status immediately in the UI
    const droneList = document.getElementById('drone-list');
    const droneItem = droneList.querySelector(`[data-drone-id="${droneId}"]`);
    if (droneItem) {
        droneItem.querySelector('.active-command').textContent = command;
    }
    
    // Send the command to the server
    socket.emit('send_command', {
        drone_id: droneId,
        command: command
    });
}); 