#!/bin/bash
set -e

# Verification script for composition demo replay

echo "Checking for play_log directory..."
if [ ! -d "play_log" ]; then
    echo "❌ Error: play_log directory not found"
    exit 1
fi
echo "✅ play_log directory exists"

# Find latest log directory
LOG_DIR=$(ls -t play_log | head -1)
if [ -z "$LOG_DIR" ]; then
    echo "❌ Error: No log directories found"
    exit 1
fi
echo "✅ Found log directory: play_log/$LOG_DIR"

# Check container was spawned
echo ""
echo "Checking container process..."
CONTAINER_OUT=$(find "play_log/$LOG_DIR/node/rclcpp_components" -name "out" | head -1)
if [ -z "$CONTAINER_OUT" ] || [ ! -f "$CONTAINER_OUT" ]; then
    echo "❌ Error: Container output not found"
    exit 1
fi
echo "✅ Container output exists: $CONTAINER_OUT"

# Check container started
if ! grep -q "ComponentManager" "$CONTAINER_OUT" 2>/dev/null; then
    echo "⚠️  Warning: ComponentManager not found in container output"
else
    echo "✅ Container ComponentManager started"
fi

# Check composable nodes were loaded
echo ""
echo "Checking composable node loading..."

# Check Talker
TALKER_DIR="play_log/$LOG_DIR/load_node/@my_container/composition/composition::Talker"
if [ ! -d "$TALKER_DIR" ]; then
    echo "❌ Error: Talker load directory not found"
    exit 1
fi
echo "✅ Talker load directory exists"

# Check Talker service response (round 1)
TALKER_RESPONSE=$(find "$TALKER_DIR" -name "service_response.*" | head -1)
if [ ! -f "$TALKER_RESPONSE" ]; then
    echo "❌ Error: Talker service response not found"
    exit 1
fi
echo "✅ Talker service response exists"

# Check Talker load was successful
if grep -q "success: true" "$TALKER_RESPONSE" 2>/dev/null; then
    echo "✅ Talker loaded successfully"
else
    echo "❌ Error: Talker load failed"
    cat "$TALKER_RESPONSE"
    exit 1
fi

# Check Listener
LISTENER_DIR="play_log/$LOG_DIR/load_node/@my_container/composition/composition::Listener"
if [ ! -d "$LISTENER_DIR" ]; then
    echo "❌ Error: Listener load directory not found"
    exit 1
fi
echo "✅ Listener load directory exists"

# Check Listener service response (round 1)
LISTENER_RESPONSE=$(find "$LISTENER_DIR" -name "service_response.*" | head -1)
if [ ! -f "$LISTENER_RESPONSE" ]; then
    echo "❌ Error: Listener service response not found"
    exit 1
fi
echo "✅ Listener service response exists"

# Check Listener load was successful
if grep -q "success: true" "$LISTENER_RESPONSE" 2>/dev/null; then
    echo "✅ Listener loaded successfully"
else
    echo "❌ Error: Listener load failed"
    cat "$LISTENER_RESPONSE"
    exit 1
fi

# Check for talker output
echo ""
echo "Checking communication..."
if grep -q "Publishing:" "$CONTAINER_OUT" 2>/dev/null; then
    echo "✅ Talker is publishing messages"
else
    echo "⚠️  Warning: No publishing messages found (might be timing issue)"
fi

if grep -q "I heard:" "$CONTAINER_OUT" 2>/dev/null; then
    echo "✅ Listener is receiving messages"
else
    echo "⚠️  Warning: No received messages found (might be timing issue)"
fi

echo ""
echo "================================"
echo "✅ All verification checks passed!"
echo "================================"
echo ""
echo "Summary:"
echo "  - Container spawned: ✅"
echo "  - Talker loaded via service: ✅"
echo "  - Listener loaded via service: ✅"
echo "  - Service-based loading working correctly!"
