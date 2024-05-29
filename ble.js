// Adding an event listener to the 'connect' button for the 'click' event
document.getElementById('connect').addEventListener('click', function() {
    // Declare a variable to hold the Bluetooth GATT Characteristic object
    let characteristic;

    // Requesting a Bluetooth device that provides a specific service
    navigator.bluetooth.requestDevice({
        filters: [{ services: ['4fafc201-1fb5-459e-8fcc-c5c9c331914b'] }]
    })
    .then(device => {
        // Log the connection attempt to the console
        console.log('Connecting to device...');
        // Connect to the device's GATT Server
        return device.gatt.connect();
    })
    .then(server => {
        // Log that the service is being retrieved
        console.log('Getting service...');
        // Get the primary service from the GATT server
        return server.getPrimaryService('4fafc201-1fb5-459e-8fcc-c5c9c331914b');
    })
    .then(service => {
        // Log that the characteristic is being retrieved
        console.log('Getting characteristic...');
        // Get a specific characteristic from the service
        return service.getCharacteristic('beb5483e-36e1-4688-b7f5-ea07361b26a8');
    })
    .then(char => {
        // Assign the retrieved characteristic to the variable
        characteristic = char;
        // Log successful connection
        console.log('Connected!');
        // Update the webpage to show the connection status
        document.getElementById('status').textContent = 'Connected';
        // Enable the 'disconnect' button
        document.getElementById('disconnect').disabled = false;

        // Log that a specific checkpoint has been reached
        console.log('Reached CP1!');
        // Setup to receive notifications from the characteristic
        characteristic.startNotifications().then(_ => {
            // Log that notifications have started
            console.log('Notifications started');
            // Add an event listener to handle changes in the characteristic's value
            characteristic.addEventListener('characteristicvaluechanged', handleCharacteristicValueChanged);
        });

        // Optionally read the initial value of the characteristic
        return characteristic.readValue();
    })
    .then(value => {
        // Check if there is a value to decode and display
        if (value) {
            // Log and display the initial characteristic value, decoding it as text
            console.log('Initial value:', new TextDecoder().decode(value));
            document.getElementById('data').textContent = new TextDecoder().decode(value);
        }
    })
    .catch(error => {
        // Log and display any errors during connection
        console.error('Connection failed!', error);
        document.getElementById('status').textContent = 'Disconnected';
    });

    // Define a function to handle changes in the characteristic's value
    function handleCharacteristicValueChanged(event) {
        // Create an array from the characteristic's value
        const data = new Uint8Array(event.target.value.buffer);
        // Convert the bytes to an integer
        const value = data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
        // Log and display the updated value
        console.log('Updated value:', value);
        document.getElementById('data').textContent = value;
    }
});

// Adding an event listener to the 'disconnect' button for the 'click' event
document.getElementById('disconnect').addEventListener('click', function() {
    // Check if the characteristic exists and is connected
    if (characteristic && characteristic.service.device.gatt.connected) {
        // Disconnect from the device's GATT server
        characteristic.service.device.gatt.disconnect();
        // Log the disconnection
        console.log('Disconnected');
        // Update the webpage to show the disconnection status
        document.getElementById('status').textContent = 'Disconnected';
        // Disable the 'disconnect' button
        document.getElementById('disconnect').disabled = true;
        // Remove the event listener for characteristic value changes
        characteristic.removeEventListener('characteristicvaluechanged', handleCharacteristicValueChanged);
    }
});
