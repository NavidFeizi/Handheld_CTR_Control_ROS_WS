# Qt Master Node Refactoring Guide

## Overview
The qt_master_node.cpp file has been refactored to improve code organization and readability by separating Qt GUI-related code from ROS2 business logic.

## New File Structure

### 1. Header Files (include/manager/)
- **qt_master_node.hpp**: Main MasterNode class declaration with ROS2 functionality
- **qt_gui.hpp**: QtGuiManager class for all GUI-related operations

### 2. Source Files (src/)
- **qt_master_node.cpp**: Contains ROS2 logic, control algorithms, and business logic (NEEDS TO BE UPDATED)
- **qt_gui.cpp**: Contains all Qt widget creation, layout, and table update functions

### 3. Updated Files
- **CMakeLists.txt**: Updated to compile both source files and include the new header directory

## What Was Separated

### Moved to qt_gui.cpp/hpp:
1. **Widget Creation**:
   - Robot status table
   - EM tracker table  
   - Planner status table
   - Control buttons (radio buttons, checkboxes)
   - Procedure buttons (freeze, start, end, insert, retract)
   - Main buttons (enable, find, engage, etc.)

2. **Table Update Functions**:
   - `updateRobotStatusTable()`
   - `updateEmtTable()`
   - `updatePlannerStatusTable()`

3. **GUI Initialization**:
   - All `initGui()` functionality moved to `QtGuiManager::initializeGui()`
   - Separated into helper functions:
     - `createTables()`
     - `createControlButtons()`
     - `createMainButtons()`
     - `connectSignals()`

### Remains in qt_master_node.cpp:
1. **ROS2 Functionality**:
   - All ROS2 subscriptions, publishers, clients
   - Callback functions
   - Service handlers
   
2. **Control Logic**:
   - `command()` - Main control loop
   - `publish_position()` / `publish_velocity()`
   - Planner response handling
   
3. **Utility Functions**:
   - CSV file reading
   - Configuration list adjustment
   - Reach status checking

## Next Steps to Complete Migration

### Step 1: Update qt_master_node.cpp
You need to replace the current monolithic qt_master_node.cpp with a streamlined version that:

1. Includes the new headers:
```cpp
#include "manager/qt_master_node.hpp"
#include "manager/qt_gui.hpp"
```

2. Constructor initializes GUI manager:
```cpp
MasterNode::MasterNode(QWidget* parent)
    : QWidget(parent)
    , rclcpp::Node("qt_master_node")
    , m_gui_manager(std::make_unique<QtGuiManager>(this))
{
    m_gui_manager->initializeGui();
    initRosInterfaces();
    m_high_level_mode = HighLvlCtrMode::Planner;
}
```

3. Removes all GUI creation code (already in qt_gui.cpp)
4. Removes table update functions (already in qt_gui.cpp)
5. Keeps all ROS2 callbacks and control logic

### Step 2: Add Public Helper Methods
The following methods need to be made public (or added) in MasterNode for GUI callbacks:

```cpp
// Public methods for GUI callbacks
void sendConfigCommand(const std::string& command, bool use_enable_service);
void handleFreezeButtonClicked(QPushButton* freeze_button);
void handleAutoInsertClicked();
void handleAutoRetractClicked();
```

### Step 3: Extract Button Click Logic
Move the lambda functions from button connections into named methods:

Example:
```cpp
void MasterNode::handleAutoInsertClicked()
{
    if (m_high_level_mode == HighLvlCtrMode::Deployment)
    {
        m_auto_insert = !m_auto_insert;
        if (m_auto_insert) {
            m_auto_retract = false;
        }
        RCLCPP_INFO(this->get_logger(), "Auto-insert mode: %s", 
                    m_auto_insert ? "ON" : "OFF");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), 
                    "Auto-insert only available in Deployment mode");
    }
}
```

## Benefits

1. **Improved Readability**: 
   - GUI code separate from business logic
   - Each file has clear responsibility

2. **Easier Maintenance**:
   - GUI changes don't affect ROS2 logic
   - Easier to find and modify specific functionality

3. **Better Organization**:
   - ~1100 line file split into manageable pieces
   - Header files provide clear API

4. **Reusability**:
   - QtGuiManager could be reused or mocked for testing
   - Business logic testable independently

## File Sizes (Approximate)
- Original qt_master_node.cpp: ~1150 lines
- New qt_master_node.cpp: ~600 lines (ROS2 + control logic)
- New qt_gui.cpp: ~350 lines (GUI creation + updates)
- Headers: ~200 lines total

## Migration Checklist
- [x] Create qt_gui.hpp header
- [x] Create qt_gui.cpp implementation  
- [x] Create qt_master_node.hpp header
- [x] Update CMakeLists.txt
- [ ] Update qt_master_node.cpp to use new structure
- [ ] Test compilation
- [ ] Test runtime functionality
- [ ] Verify all GUI interactions work
- [ ] Verify all ROS2 functionality works
