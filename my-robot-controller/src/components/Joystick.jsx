// src/components/Joystick.jsx
import React, { useState, useRef, useEffect } from 'react';
import './joystick.css';


const Joystick = ({ onMove, onStop, speed }) => {
    const [dragging, setDragging] = useState(false);
    const knobRef = useRef(null);
    const baseRef = useRef(null);
    const joyInfoRef = useRef(null);

    const sendCommand = (x, y) => {
        const baseSpeed = speed / 100;
        const right= Math.round((y + (-x)) * baseSpeed * 100);
        const left = Math.round((y + x) * baseSpeed * 100);
        if (joyInfoRef.current) {
            joyInfoRef.current.textContent = `L:${left} R:${right}`;
        }
        onMove(left, right);
    };

    const handlePointerMove = (e) => {
        if (!dragging || !baseRef.current || !knobRef.current) return;
        
        const joyRect = baseRef.current.getBoundingClientRect();
        const cx = joyRect.width / 2;
        const cy = joyRect.height / 2;
        const maxRadius = (joyRect.width / 2) - (knobRef.current.offsetWidth / 2);
        
        const dx = e.clientX - (joyRect.left + cx);
        const dy = e.clientY - (joyRect.top + cy);
        
        const dist = Math.sqrt(dx * dx + dy * dy);
        const r = Math.min(dist, maxRadius);
        const angle = Math.atan2(dy, dx);
        
        const nx = r * Math.cos(angle);
        const ny = r * Math.sin(angle);
        
        knobRef.current.style.transform = `translate(calc(-50% + ${nx}px), calc(-50% + ${ny}px))`;
        
        const nxNorm = nx / maxRadius;
        const nyNorm = -ny / maxRadius; // Yを反転
        
        sendCommand(nxNorm, nyNorm);
    };

    const handlePointerUp = (e) => {
        if (!dragging) return;
        setDragging(false);
        try { baseRef.current.releasePointerCapture(e.pointerId); } catch (err) {}
        
        knobRef.current.style.transform = 'translate(-50%, -50%)';
        if(joyInfoRef.current) joyInfoRef.current.textContent = 'Neutral';
        onStop();
    };
    const handlePointerDown = (e) => {
        e.preventDefault();
        setDragging(true);
        baseRef.current.setPointerCapture(e.pointerId);
    };

    useEffect(() => {
    window.addEventListener('pointermove', handlePointerMove);
    window.addEventListener('pointerup', handlePointerUp);

    return () => {
        window.removeEventListener('pointermove', handlePointerMove);
        window.removeEventListener('pointerup', handlePointerUp);
    };
  }, [dragging, speed]); // draggingとspeedが変わるたびにイベントリスナーを再設定

    return (
    <div className="controls-wrapper">
        <div>
            <div className="joystick-container" ref={baseRef} onPointerDown={handlePointerDown}>
                <div className="joy-base">
                <div className="joy-knob" ref={knobRef}></div>
            </div>
            </div>
            <div className="joy-info" ref={joyInfoRef}>Neutral</div>
        </div>
        <div>
            <button className="stop-btn-small" onClick={onStop}>Stop</button>
        </div>
        </div>
    );
};

export default Joystick;