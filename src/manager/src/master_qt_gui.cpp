#include "manager/master_qt_gui.hpp"
#include "manager/master_node.hpp"

#include <cmath>

QtGuiManager::QtGuiManager(MasterNode* master_node)
    : m_master_node(master_node)
{
}

void QtGuiManager::initializeGui()
{
    m_master_node->setWindowTitle("Handheld CTR Master");
    m_master_node->setGeometry(200, 200, 550, 800);
    m_master_node->setFocusPolicy(Qt::StrongFocus);
    
    QVBoxLayout* layout = new QVBoxLayout(m_master_node);
    
    createTables(layout);
    createControlButtons(layout);
    createMainButtons(layout);
    connectSignals();
    
    m_master_node->setLayout(layout);
}

void QtGuiManager::createTables(QVBoxLayout* layout)
{
    // Robot info table
    QLabel* tableTitle = new QLabel("Robot info", m_master_node);
    tableTitle->setAlignment(Qt::AlignCenter);
    layout->addWidget(tableTitle);
    
    mp_robot_status_table = new QTableWidget(1, 6, m_master_node);
    mp_robot_status_table->setHorizontalHeaderLabels({"Head", "Enable", "Procedure", "Engaged", "Lock", "EStop"});
    mp_robot_status_table->setVerticalHeaderLabels({" Status  "});
    mp_robot_status_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    mp_robot_status_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    mp_robot_status_table->setFixedHeight(50);
    layout->addWidget(mp_robot_status_table);
    
    // Cartesian Space table
    tableTitle = new QLabel("Cartesian Space", m_master_node);
    tableTitle->setAlignment(Qt::AlignCenter);
    layout->addWidget(tableTitle);
    
    mp_emt_status_table = new QTableWidget(7, 5, m_master_node);
    mp_emt_status_table->setHorizontalHeaderLabels({"x", "y", "z", "norm", "theta"});
    mp_emt_status_table->setVerticalHeaderLabels({" Tip ", " PINNs ", "PINNs-Tip Error ", " Probe ", " Probe-Tip error ", " CSV Target ", " CSV-Tip Error "});
    mp_emt_status_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    mp_emt_status_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    mp_emt_status_table->setFixedHeight(250);
    layout->addWidget(mp_emt_status_table);
    
    // Planner status table
    tableTitle = new QLabel("Planner Status", m_master_node);
    tableTitle->setAlignment(Qt::AlignCenter);
    layout->addWidget(tableTitle);
    
    mp_planner_status_table = new QTableWidget(1, 3, m_master_node);
    mp_planner_status_table->setHorizontalHeaderLabels({"Status", "Success", "IK Error"});
    mp_planner_status_table->setVerticalHeaderLabels({" Planner  "});
    mp_planner_status_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    mp_planner_status_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    mp_planner_status_table->setFixedHeight(50);
    layout->addWidget(mp_planner_status_table);
}

void QtGuiManager::createControlButtons(QVBoxLayout* layout)
{
    QLabel* tableTitle = new QLabel("Controls", m_master_node);
    tableTitle->setAlignment(Qt::AlignCenter);
    layout->addWidget(tableTitle);
    
    QHBoxLayout* bulletLayoutGroups = new QHBoxLayout();
    
    // Radio button group for controller mode
    mp_mode_group_box = new QGroupBox("Mode", m_master_node);
    mp_mode_layout = new QVBoxLayout();
    mp_mode_radio_planner = new QRadioButton("Select Target", m_master_node);
    mp_mode_radio_deployment = new QRadioButton("Deployment", m_master_node);
    
    mp_ctrl_mode_group = new QButtonGroup(m_master_node);
    mp_ctrl_mode_group->addButton(mp_mode_radio_planner, 0);
    mp_ctrl_mode_group->addButton(mp_mode_radio_deployment, 1);
    mp_ctrl_mode_group->setExclusive(true);
    
    mp_mode_layout->addWidget(mp_mode_radio_planner);
    mp_mode_layout->addWidget(mp_mode_radio_deployment);
    mp_mode_group_box->setLayout(mp_mode_layout);
    mp_mode_radio_planner->setChecked(true);
    
    // Closed loop checkbox
    mp_closed_loop_checkbox = new QCheckBox("Closed Loop", m_master_node);
    mp_closed_loop_checkbox->setEnabled(true);
    mp_mode_layout->addWidget(mp_closed_loop_checkbox);
    
    bulletLayoutGroups->addWidget(mp_mode_group_box);
    
    // Procedure buttons
    QVBoxLayout* procedureLayoutGroup = new QVBoxLayout();
    QPushButton* freeze_button = new QPushButton("Freeze Robot", m_master_node);
    freeze_button->setStyleSheet("background-color: rgb(255, 0, 0); color: white;");
    QPushButton* start_button = new QPushButton("Start Procedure", m_master_node);
    QPushButton* end_button = new QPushButton("End Procedure", m_master_node);
    QPushButton* insert_button = new QPushButton("Auto Insert", m_master_node);
    QPushButton* retract_button = new QPushButton("Auto Retract", m_master_node);
    
    procedureLayoutGroup->addWidget(freeze_button);
    procedureLayoutGroup->addWidget(start_button);
    procedureLayoutGroup->addWidget(end_button);
    procedureLayoutGroup->addWidget(insert_button);
    procedureLayoutGroup->addWidget(retract_button);
    
    bulletLayoutGroups->addLayout(procedureLayoutGroup);
    layout->addLayout(bulletLayoutGroups);
    
    // Connect procedure buttons
    QObject::connect(freeze_button, &QPushButton::clicked, m_master_node, 
        [this, freeze_button]() { m_master_node->handleFreezeButtonClicked(freeze_button); });
    
    QObject::connect(start_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("startProcedure", false); });
    
    QObject::connect(end_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("endProcedure", false); });
    
    QObject::connect(insert_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->handleAutoInsertClicked(); });
    
    QObject::connect(retract_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->handleAutoRetractClicked(); });
}

void QtGuiManager::createMainButtons(QVBoxLayout* layout)
{
    // Add automated test button at top (blue styling)
    QPushButton* test_button = new QPushButton("Run Auto Test", m_master_node);
    test_button->setStyleSheet("background-color: rgb(0, 150, 255); color: white; font-weight: bold;");
    layout->addWidget(test_button);
    
    QObject::connect(test_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->handleTestButtonClicked(); });
    
    // Manual target selection buttons
    QHBoxLayout* target_button_layout = new QHBoxLayout();
    
    // Toggle target mode button
    mp_toggle_target_mode_button = new QPushButton("Toggle: Probe Mode", m_master_node);
    target_button_layout->addWidget(mp_toggle_target_mode_button);
    QObject::connect(mp_toggle_target_mode_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->handleToggleTargetModeClicked(); });
    
    // Previous target button
    mp_prev_target_button = new QPushButton("◄ Previous", m_master_node);
    mp_prev_target_button->setEnabled(false);  // Disabled by default (Probe mode)
    target_button_layout->addWidget(mp_prev_target_button);
    QObject::connect(mp_prev_target_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->handlePrevTargetClicked(); });
    
    // Next target button
    mp_next_target_button = new QPushButton("Next ►", m_master_node);
    mp_next_target_button->setEnabled(false);  // Disabled by default (Probe mode)
    target_button_layout->addWidget(mp_next_target_button);
    QObject::connect(mp_next_target_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->handleNextTargetClicked(); });
    
    layout->addLayout(target_button_layout);
    
    // Add space
    layout->addSpacing(20);
    
    // Robot Control section
    QPushButton* enable_button = new QPushButton("Disable", m_master_node);
    QPushButton* find_button = new QPushButton("Find", m_master_node);
    mp_engage_button = new QPushButton("Engage and Unlock", m_master_node);
    mp_disengage_button = new QPushButton("Lock and Disengage", m_master_node);
    QPushButton* setRotaryHome_button = new QPushButton("Find Rotary Home and Go Home and Start Procedure", m_master_node);
    QPushButton* home_button = new QPushButton("Go Home", m_master_node);
    
    mp_engage_button->setEnabled(true);
    mp_disengage_button->setEnabled(true);
    
    QHBoxLayout* buttonRow_0 = new QHBoxLayout();
    buttonRow_0->addWidget(mp_engage_button);
    buttonRow_0->addWidget(mp_disengage_button);
    
    layout->addWidget(enable_button);
    layout->addWidget(find_button);
    layout->addLayout(buttonRow_0);
    layout->addWidget(setRotaryHome_button);
    layout->addWidget(home_button);
    
    // Connect main buttons
    QObject::connect(enable_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("toggleEnable", true); });
    
    QObject::connect(find_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("findLinearHome", false); });
    
    QObject::connect(mp_engage_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("engageAndUnlock", false); });
    
    QObject::connect(mp_disengage_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("lockAndDisengage", false); });
    
    QObject::connect(setRotaryHome_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("findRotaryHomeAndGoHome", false); });
    
    QObject::connect(home_button, &QPushButton::clicked, m_master_node,
        [this]() { m_master_node->sendConfigCommand("goHome", false); });
    
    // Connect signals for text updates
    QObject::connect(m_master_node, &MasterNode::update_enable_button_Text, 
        enable_button, &QPushButton::setText);
    
    QObject::connect(m_master_node, &MasterNode::update_wrench_pos_button_Text,
        mp_engage_button, &QPushButton::setText);
}

void QtGuiManager::connectSignals()
{
    QObject::connect(mp_ctrl_mode_group, QOverload<int>::of(&QButtonGroup::idClicked),
        m_master_node, &MasterNode::onCtrlModeClicked);
    
    QObject::connect(mp_closed_loop_checkbox, &QCheckBox::toggled,
        m_master_node, &MasterNode::onClosedLoopToggled);
    
    QObject::connect(m_master_node, &MasterNode::robotStatusUpdated,
        m_master_node, [this](bool en, bool proc, bool head, bool att, int lock) {
            updateRobotStatusTable(en, proc, head, att, lock);
        }, Qt::QueuedConnection);
    
    QObject::connect(m_master_node, &MasterNode::emtUpdated,
        m_master_node, [this](double tx, double ty, double tz, double px, double py, double pz, 
                              double pinnx, double pinny, double pinnz) {
            updateEmtTable(Eigen::Vector3d(tx, ty, tz),
                          Eigen::Vector3d(px, py, pz),
                          Eigen::Vector3d(pinnx, pinny, pinnz));
        }, Qt::QueuedConnection);
    
    QObject::connect(m_master_node, &MasterNode::plannerStatusUpdated,
        m_master_node, [this](bool planning, bool success, double ik_error) {
            updatePlannerStatusTable(planning, success, ik_error);
        }, Qt::QueuedConnection);
}

void QtGuiManager::updateRobotStatusTable(bool enable, bool procedure, bool head, bool attached, int locked)
{
    mp_robot_status_table->setItem(0, 0, new QTableWidgetItem(head ? "Attached" : "Detached"));
    mp_robot_status_table->setItem(0, 1, new QTableWidgetItem(enable ? "ON" : "OFF"));
    mp_robot_status_table->setItem(0, 2, new QTableWidgetItem(procedure ? "ON" : "OFF"));
    mp_robot_status_table->setItem(0, 3, new QTableWidgetItem(attached ? "Yes" : "No"));
    
    if (locked == 1)
    {
        mp_robot_status_table->setItem(0, 4, new QTableWidgetItem("Locked"));
    }
    else if (locked == -1)
    {
        mp_robot_status_table->setItem(0, 4, new QTableWidgetItem("Unlocked"));
    }
    else
    {
        mp_robot_status_table->setItem(0, 4, new QTableWidgetItem("Unknown"));
    }
}

void QtGuiManager::updateEmtTable(Eigen::Vector3d tip, Eigen::Vector3d probe, Eigen::Vector3d pinns)
{
    mp_emt_status_table->setItem(0, 0, new QTableWidgetItem(QString::number(tip[0], 'f', 3)));
    mp_emt_status_table->setItem(0, 1, new QTableWidgetItem(QString::number(tip[1], 'f', 3)));
    mp_emt_status_table->setItem(0, 2, new QTableWidgetItem(QString::number(tip[2], 'f', 3)));
    mp_emt_status_table->setItem(0, 3, new QTableWidgetItem(QString::number(tip.norm(), 'f', 3)));
    mp_emt_status_table->setItem(0, 4, new QTableWidgetItem(QString::number(atan2(tip[1], tip[0]), 'f', 3)));
    
    mp_emt_status_table->setItem(1, 0, new QTableWidgetItem(QString::number(pinns[0], 'f', 3)));
    mp_emt_status_table->setItem(1, 1, new QTableWidgetItem(QString::number(pinns[1], 'f', 3)));
    mp_emt_status_table->setItem(1, 2, new QTableWidgetItem(QString::number(pinns[2], 'f', 3)));
    mp_emt_status_table->setItem(1, 3, new QTableWidgetItem(QString::number(pinns.norm(), 'f', 3)));
    mp_emt_status_table->setItem(1, 4, new QTableWidgetItem(QString::number(atan2(pinns[1], pinns[0]), 'f', 3)));
    
    Eigen::Vector3d error_pinns = tip - pinns;
    mp_emt_status_table->setItem(2, 0, new QTableWidgetItem(QString::number(error_pinns[0], 'f', 3)));
    mp_emt_status_table->setItem(2, 1, new QTableWidgetItem(QString::number(error_pinns[1], 'f', 3)));
    mp_emt_status_table->setItem(2, 2, new QTableWidgetItem(QString::number(error_pinns[2], 'f', 3)));
    mp_emt_status_table->setItem(2, 3, new QTableWidgetItem(QString::number(error_pinns.norm(), 'f', 3)));
    
    mp_emt_status_table->setItem(3, 0, new QTableWidgetItem(QString::number(probe[0], 'f', 3)));
    mp_emt_status_table->setItem(3, 1, new QTableWidgetItem(QString::number(probe[1], 'f', 3)));
    mp_emt_status_table->setItem(3, 2, new QTableWidgetItem(QString::number(probe[2], 'f', 3)));
    mp_emt_status_table->setItem(3, 3, new QTableWidgetItem(QString::number(probe.norm(), 'f', 3)));
    mp_emt_status_table->setItem(3, 4, new QTableWidgetItem(QString::number(atan2(probe[1], probe[0]), 'f', 3)));
    
    Eigen::Vector3d error = tip - probe;
    mp_emt_status_table->setItem(4, 0, new QTableWidgetItem(QString::number(error[0], 'f', 3)));
    mp_emt_status_table->setItem(4, 1, new QTableWidgetItem(QString::number(error[1], 'f', 3)));
    mp_emt_status_table->setItem(4, 2, new QTableWidgetItem(QString::number(error[2], 'f', 3)));
    mp_emt_status_table->setItem(4, 3, new QTableWidgetItem(QString::number(error.norm(), 'f', 3)));
}

void QtGuiManager::updatePlannerStatusTable(bool planning, bool success, double ik_error)
{
    // Create status item and color "Planning" orange
    QTableWidgetItem* status_item = new QTableWidgetItem(planning ? "Planning" : "Idle");
    if (planning)
    {
        status_item->setForeground(QBrush(QColor(255, 140, 0))); // Orange text
    }
    mp_planner_status_table->setItem(0, 0, status_item);
    
    // Create success item and color "No" red
    QTableWidgetItem* success_item = new QTableWidgetItem(success ? "Yes" : "No");
    if (!success)
    {
        success_item->setForeground(QBrush(QColor(255, 0, 0))); // Red text
    }
    mp_planner_status_table->setItem(0, 1, success_item);
    
    // Create IK error item and color it red if error exceeds threshold
    QTableWidgetItem* ik_error_item = new QTableWidgetItem(QString::number(ik_error, 'f', 4));
    if (ik_error > 0.003)
    {
        ik_error_item->setForeground(QBrush(QColor(255, 0, 0))); // Red text
    }
    mp_planner_status_table->setItem(0, 2, ik_error_item);
}
