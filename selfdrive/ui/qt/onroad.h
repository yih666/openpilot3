#pragma once

#include <QStackedLayout>
#include <QWidget>
#include <QPushButton>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"

#ifdef QCOM2
#include <QTimer>
#include <QMap>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"
#endif


// ***** onroad widgets *****

class ButtonsWindow : public QWidget {
  Q_OBJECT

public:
  ButtonsWindow(QWidget* parent = 0);

private:
  QPushButton *dlpBtn;

  //int dlpStatus = -1;
  const QStringList dlpBtnColors = {"#87ceeb", "#00ff16", "#808080"};

public slots:
  void updateState(const UIState &s);
};

class OnroadHud : public QWidget {
  Q_OBJECT
  Q_PROPERTY(bool engageable MEMBER engageable NOTIFY valueChanged); 
  Q_PROPERTY(int status MEMBER status NOTIFY valueChanged);
  Q_PROPERTY(float ang_str MEMBER ang_str NOTIFY valueChanged);
  Q_PROPERTY(bool compass MEMBER compass);
  Q_PROPERTY(float bearingDeg MEMBER bearingDeg);
  Q_PROPERTY(float bearingAccuracyDeg MEMBER bearingAccuracyDeg);

public:
  explicit OnroadHud(QWidget *parent);
  void updateState(const UIState &s);

private:
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, bool rotation = false, float angle = 0 );
  //void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  //void drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color);
  void drawCompass(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, float bearing_deg = 0);
  void paintEvent(QPaintEvent *event) override;
  
  QPixmap engage_img;
  QPixmap compass_inner_img;
  QPixmap compass_outer_img;
  const int radius = 192;
  const int img_size = (radius / 2) * 1.5;
  bool engageable = false;
  int status = STATUS_DISENGAGED;
  float ang_str = 0;
  bool compass;
  float bearingDeg = 0;
  float bearingAccuracyDeg;

protected:
  inline QColor blackColor(int alpha = 200) { return QColor(0, 0, 0, alpha); }
  
signals:
  void valueChanged();
};

class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a, const QColor &color);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};
};

// container window for the NVG UI
class NvgWindow : public CameraViewWidget {
  Q_OBJECT

public:
  explicit NvgWindow(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);
  OnroadHud *hud;
 
private:
  void ui_draw_line(QPainter &painter, const line_vertices_data &vd);
  
protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool is_radar);
  inline QColor redColor(int alpha = 200) { return QColor(201, 34, 49, alpha); }
  inline QColor blackColor(int alpha = 200) { return QColor(0, 0, 0, alpha); }
  inline QColor greenColor(int alpha = 200) { return QColor(49, 201, 34, alpha); }
  inline QColor graceBlueColor(int alpha = 200) { return QColor (34, 49, 201, alpha); }
  inline QColor skyBlueColor(int alpha = 200) { return QColor (135, 206, 230, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor tomatoColor(int alpha = 250) { return QColor (255, 99, 71, alpha); }
  inline QColor yellowColor(int alpha = 255) { return QColor(255, 255, 0, alpha); }
  inline QColor orangeColor(int alpha = 255) { return QColor(255, 165, 0, alpha); }
  
  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
  
  uint64_t last_update_params;
  
  // neokii
  QPixmap ic_brake;
  QPixmap ic_autohold_warning;
  QPixmap ic_autohold_active;
  QPixmap ic_nda;
  QPixmap ic_hda;
  QPixmap ic_tire_pressure;
  QPixmap ic_turn_signal_l;
  QPixmap ic_turn_signal_r;
  QPixmap ic_satellite;
  QPixmap ic_scc2;
  QPixmap ic_radar;
  QPixmap ic_radar_vision;
  
  QMap<QString, QPixmap> ic_oil_com;
  
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color);
  void drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color);
  void drawMaxSpeed(QPainter &p);
  void drawSpeed(QPainter &p);
  void drawBottomIcons(QPainter &p);
  void drawTurnSignals(QPainter &p);
  void drawDebugText(QPainter &p);
  void drawCgear(QPainter &p);//기어
  void drawTpms(QPainter &p);
  void drawBrake(QPainter &p);
  void drawCommunity(QPainter &p);
  void drawGpsStatus(QPainter &p);
  void drawSteer(QPainter &p);
  void drawEngRpm(QPainter &p);
  
  const int radius = 192;
  const int img_size = (radius / 2) * 1.5;

  float m_cur_speed = 0;
  
signals:
  void resizeSignal(int w);  
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

protected:
  void mousePressEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  
  void paintEvent(QPaintEvent *event) override;

private:
  OnroadHud *hud;
  OnroadAlerts *alerts;
  NvgWindow *nvg;
  ButtonsWindow *buttons;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;

  // neokii
#ifdef QCOM2  
private:
  ScreenRecoder* recorder;
  std::shared_ptr<QTimer> record_timer;
  QPoint startPos;
#endif  

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
