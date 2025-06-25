import React, { useState, useEffect } from 'react';

const BACKEND_URL = 'http://192.168.148.191:5000';

const App: React.FC = () => {
  const [images, setImages] = useState<string[]>([]);
  const [isRunning, setIsRunning] = useState(false);
  const [statusMessage, setStatusMessage] = useState('');

  const startDrone = async () => {
    try {
      const response = await fetch(`${BACKEND_URL}/start`, { method: 'POST' });
      if (response.ok) {
        setIsRunning(true);
        setStatusMessage('Drone started');
      } else {
        const data = await response.json();
        setStatusMessage(`Failed to start the drone: ${data.message || 'Unknown error'}`);
      }
    } catch (error) {
      setStatusMessage('Error starting the drone');
    }
  };

  const stopDrone = async () => {
    try {
      const response = await fetch(`${BACKEND_URL}/stop`, { method: 'POST' });
      if (response.ok) {
        setIsRunning(false);
        setStatusMessage('Drone stopped');
      } else {
        const data = await response.json();
        setStatusMessage(`Failed to stop the drone: ${data.message || 'Unknown error'}`);
      }
    } catch (error) {
      setStatusMessage('Error stopping the drone');
    }
  };

  const fetchImages = async () => {
    try {
      const response = await fetch(`${BACKEND_URL}/images`);
      if (response.ok) {
        const data = await response.json();
        setImages(data.images || []);
      } else {
        setStatusMessage('Failed to fetch images');
      }
    } catch (error) {
      setStatusMessage('Error fetching images');
    }
  };

  useEffect(() => {
    fetchImages();
  }, []);

  return (
    <div style={{ padding: '20px', fontFamily: 'Arial, sans-serif', backgroundColor: '#f0f8ff', color: '#333' }}>
      <h1 style={{ color: '#007acc' }}>Underwater Drone Dashboard</h1>
      <div style={{ marginBottom: '10px', minHeight: '24px' }}>{statusMessage}</div>
      <div style={{ marginBottom: '20px' }}>
        <button
          onClick={startDrone}
          disabled={isRunning}
          style={{
            marginRight: '10px',
            backgroundColor: isRunning ? '#a0a0a0' : '#28a745',
            color: 'white',
            border: 'none',
            padding: '10px 20px',
            cursor: isRunning ? 'not-allowed' : 'pointer',
            borderRadius: '5px',
          }}
        >
          Start
        </button>
        <button
          onClick={stopDrone}
          disabled={!isRunning}
          style={{
            backgroundColor: !isRunning ? '#a0a0a0' : '#dc3545',
            color: 'white',
            border: 'none',
            padding: '10px 20px',
            cursor: !isRunning ? 'not-allowed' : 'pointer',
            borderRadius: '5px',
          }}
        >
          Stop
        </button>
      </div>
      <h2 style={{ color: '#007acc' }}>Latest Images from ESP32-CAM</h2>
      <div style={{ display: 'flex', flexWrap: 'wrap' }}>
        {images.length === 0 && <p>No images available</p>}
        {images.slice(0, 5).map((imgUrl, index) => (
          <img
            key={index}
            src={`${BACKEND_URL}/images/${imgUrl}`}
            alt={`Captured ${index}`}
            style={{ width: '200px', height: '150px', marginRight: '10px', marginBottom: '10px', objectFit: 'cover', borderRadius: '5px', border: '1px solid #ccc' }}
          />
        ))}
      </div>
      <button
        onClick={fetchImages}
        style={{
          marginTop: '10px',
          backgroundColor: '#007acc',
          color: 'white',
          border: 'none',
          padding: '10px 20px',
          cursor: 'pointer',
          borderRadius: '5px',
        }}
      >
        Refresh Images
      </button>
    </div>
  );
};

export default App;
