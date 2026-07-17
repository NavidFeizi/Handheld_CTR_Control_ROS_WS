#ifndef QT_GUI_HPP
#define QT_GUI_HPP

#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <QObject>
#include <QString>
#include <QPushButton>
#include <QKeyEvent>
#include <QSet>
#include <QDebug>
#include <QRadioButton>
#include <QGroupBox>
#include <QtConcurrent>
#include <QButtonGroup>
#include <QCheckBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>

#include <Eigen/Dense>
#include <functional>

// Forward declaration
class MasterNode;

class QtGuiManager
{
public:
    explicit QtGuiManager(MasterNode* master_node);
    
    // Initialize the GUI
    void initializeGui();
    
    // Update table functions
    void updateRobotStatusTable(bool enable, bool procedure, bool head, bool attached, int locked);
    void updateEmtTable(Eigen::Vector3d tip, Eigen::Vector3d probe, Eigen::Vector3d pinns);
    void updatePlannerStatusTable(bool planning, bool success, double ik_error);
    
    // Getters for widgets that need to be accessed from MasterNode
    QTableWidget* getRobotStatusTable() { return mp_robot_status_table; }
    QTableWidget* getEmtStatusTable() { return mp_emt_status_table; }
    QTableWidget* getPlannerStatusTable() { return mp_planner_status_table; }
    QCheckBox* getClosedLoopCheckbox() { return mp_closed_loop_checkbox; }
    QRadioButton* getPlannerRadioButton() { return mp_mode_radio_planner; }
    QRadioButton* getDeploymentRadioButton() { return mp_mode_radio_deployment; }
    QPushButton* getToggleTargetModeButton() { return mp_toggle_target_mode_button; }
    QPushButton* getNextTargetButton() { return mp_next_target_button; }
    QPushButton* getPrevTargetButton() { return mp_prev_target_button; }

private:
    MasterNode* m_master_node;
    
    // Qt widgets
    QGroupBox* mp_mode_group_box;
    QButtonGroup* mp_ctrl_mode_group;
    QVBoxLayout* mp_mode_layout;
    QRadioButton* mp_mode_radio_planner;
    QRadioButton* mp_mode_radio_deployment;
    QCheckBox* mp_closed_loop_checkbox;
    QPushButton* mp_engage_button;
    QPushButton* mp_disengage_button;
    QPushButton* mp_toggle_target_mode_button;
    QPushButton* mp_next_target_button;
    QPushButton* mp_prev_target_button;
    QTableWidget* mp_robot_status_table;
    QTableWidget* mp_emt_status_table;
    QTableWidget* mp_planner_status_table;
    
    // Helper functions
    void createTables(QVBoxLayout* layout);
    void createControlButtons(QVBoxLayout* layout);
    void createMainButtons(QVBoxLayout* layout);
    void connectSignals();
};

#endif // QT_GUI_HPP
