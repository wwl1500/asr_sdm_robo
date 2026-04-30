#pragma once

#include <QObject>
#include <QImage>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVariant>
#include <QVariantMap>
#include <QJsonArray>
#include <QJsonObject>
#include <array>
#include <memory>
#include <mutex>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

class RosUiBridge : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString rosStatus READ rosStatus NOTIFY rosStatusChanged)

    Q_PROPERTY(QVariantMap cpuSummary READ cpuSummary NOTIFY cpuSummaryChanged)
    Q_PROPERTY(QVariantList cpuHistory READ cpuHistory NOTIFY cpuHistoryChanged)
    Q_PROPERTY(QVariantList cpuCoreRows READ cpuCoreRows NOTIFY cpuCoreRowsChanged)

    Q_PROPERTY(QVariantMap memorySummary READ memorySummary NOTIFY memorySummaryChanged)
    Q_PROPERTY(QVariantList memoryHistory READ memoryHistory NOTIFY memoryHistoryChanged)
    Q_PROPERTY(QVariantList memoryRows READ memoryRows NOTIFY memoryRowsChanged)

    Q_PROPERTY(QVariantMap hddSummary READ hddSummary NOTIFY hddSummaryChanged)
    Q_PROPERTY(QVariantList hddRows READ hddRows NOTIFY hddRowsChanged)

    Q_PROPERTY(QVariantMap netSummary READ netSummary NOTIFY netSummaryChanged)
    Q_PROPERTY(QVariantList netInHistory READ netInHistory NOTIFY netInHistoryChanged)
    Q_PROPERTY(QVariantList netOutHistory READ netOutHistory NOTIFY netOutHistoryChanged)
    Q_PROPERTY(QVariantList netInterfaceRows READ netInterfaceRows NOTIFY netInterfaceRowsChanged)

    Q_PROPERTY(QVariantMap ntpSummary READ ntpSummary NOTIFY ntpSummaryChanged)
    Q_PROPERTY(QVariantList ntpRows READ ntpRows NOTIFY ntpRowsChanged)

    Q_PROPERTY(QVariantList videoTopics READ videoTopics NOTIFY videoTopicsChanged)
    Q_PROPERTY(QVariantList videoSlots READ videoSlots NOTIFY videoSlotsChanged)
    Q_PROPERTY(QString videoTopic0 READ videoTopic0 NOTIFY videoSlot0Changed)
    Q_PROPERTY(QString videoTopic1 READ videoTopic1 NOTIFY videoSlot1Changed)
    Q_PROPERTY(QString videoTopic2 READ videoTopic2 NOTIFY videoSlot2Changed)
    Q_PROPERTY(QString videoTopic3 READ videoTopic3 NOTIFY videoSlot3Changed)
    Q_PROPERTY(QString videoStatus0 READ videoStatus0 NOTIFY videoSlot0Changed)
    Q_PROPERTY(QString videoStatus1 READ videoStatus1 NOTIFY videoSlot1Changed)
    Q_PROPERTY(QString videoStatus2 READ videoStatus2 NOTIFY videoSlot2Changed)
    Q_PROPERTY(QString videoStatus3 READ videoStatus3 NOTIFY videoSlot3Changed)
    Q_PROPERTY(int videoFrame0Revision READ videoFrame0Revision NOTIFY videoSlot0Changed)
    Q_PROPERTY(int videoFrame1Revision READ videoFrame1Revision NOTIFY videoSlot1Changed)
    Q_PROPERTY(int videoFrame2Revision READ videoFrame2Revision NOTIFY videoSlot2Changed)
    Q_PROPERTY(int videoFrame3Revision READ videoFrame3Revision NOTIFY videoSlot3Changed)

public:
    explicit RosUiBridge(QObject *parent = nullptr);
    ~RosUiBridge() override;

    QString rosStatus() const;

    QVariantMap cpuSummary() const;
    QVariantList cpuHistory() const;
    QVariantList cpuCoreRows() const;

    QVariantMap memorySummary() const;
    QVariantList memoryHistory() const;
    QVariantList memoryRows() const;

    QVariantMap hddSummary() const;
    QVariantList hddRows() const;

    QVariantMap netSummary() const;
    QVariantList netInHistory() const;
    QVariantList netOutHistory() const;
    QVariantList netInterfaceRows() const;

    QVariantMap ntpSummary() const;
    QVariantList ntpRows() const;

    QVariantList videoTopics() const;
    QVariantList videoSlots() const;
    QString videoTopic0() const;
    QString videoTopic1() const;
    QString videoTopic2() const;
    QString videoTopic3() const;
    QString videoStatus0() const;
    QString videoStatus1() const;
    QString videoStatus2() const;
    QString videoStatus3() const;
    int videoFrame0Revision() const;
    int videoFrame1Revision() const;
    int videoFrame2Revision() const;
    int videoFrame3Revision() const;
    QImage videoFrameImage(int slotIndex) const;

    Q_INVOKABLE void setVideoTopic(int slotIndex, const QString &topicName);

signals:
    void rosStatusChanged();

    void cpuSummaryChanged();
    void cpuHistoryChanged();
    void cpuCoreRowsChanged();

    void memorySummaryChanged();
    void memoryHistoryChanged();
    void memoryRowsChanged();

    void hddSummaryChanged();
    void hddRowsChanged();

    void netSummaryChanged();
    void netInHistoryChanged();
    void netOutHistoryChanged();
    void netInterfaceRowsChanged();

    void ntpSummaryChanged();
    void ntpRowsChanged();

    void videoTopicsChanged();
    void videoSlotsChanged();
    void videoSlot0Changed();
    void videoSlot1Changed();
    void videoSlot2Changed();
    void videoSlot3Changed();

private:
    void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

    static QString valueForKey(const diagnostic_msgs::msg::DiagnosticStatus &status, const QString &key);
    static double extractNumber(const QString &text);
    static QString formatNumber(double value, int precision = 1);
    static QString levelToText(unsigned char level);
    static void appendHistory(QVariantList &history, double value, int maxPoints = 60);

    void updateCpu(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateMemory(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateHdd(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateNet(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateNtp(const diagnostic_msgs::msg::DiagnosticStatus &status);

    void handleVideoTopicsMessage(const std_msgs::msg::String::SharedPtr msg);
    void handleVideoSlotStatusMessage(int slotIndex, const std_msgs::msg::String::SharedPtr msg);
    void publishVideoSelection(int slotIndex, const QString &topicName);
    void discoverVideoTopics();
    void applyDiscoveredVideoTopics(const QStringList &topicNames, const QMap<QString, QString> &topicTypes);
    void stopVideoSlot(int slotIndex);
    void startVideoSlot(int slotIndex);
    void imageCallback(int slotIndex, const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedImageCallback(int slotIndex, const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void updateVideoFrame(int slotIndex, const QImage &image, const QString &status);
    void updateVideoStatus(int slotIndex, const QString &status);
    void emitVideoSlotChanged(int slotIndex);

    static bool isPerceptionImageTopic(const QString &topicName, const QString &topicType);
    static QImage imageMessageToQImage(const sensor_msgs::msg::Image &msg, QString *errorMessage);

    struct VideoSlot {
        QString topic;
        QString topic_type;
        QString status = "No topic selected";
        int frame_revision = 0;
        QImage frame;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub;
    };

    QString ros_status_;

    QVariantMap cpu_summary_;
    QVariantList cpu_history_;
    QVariantList cpu_core_rows_;

    QVariantMap memory_summary_;
    QVariantList memory_history_;
    QVariantList memory_rows_;

    QVariantMap hdd_summary_;
    QVariantList hdd_rows_;

    QVariantMap net_summary_;
    QVariantList net_in_history_;
    QVariantList net_out_history_;
    QVariantList net_interface_rows_;

    QVariantMap ntp_summary_;
    QVariantList ntp_rows_;

    QVariantList video_topics_;
    QStringList video_topic_names_;
    QMap<QString, QString> video_topic_types_;
    QString video_topic_namespace_ = QStringLiteral("/system_monitor/video");
    std::array<VideoSlot, 4> video_slots_;
    mutable std::mutex video_frame_mutex_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr video_topics_sub_;
    rclcpp::TimerBase::SharedPtr video_topic_discovery_timer_;
    std::array<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr, 4> video_status_subs_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr video_select_pub_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_thread_;
};
