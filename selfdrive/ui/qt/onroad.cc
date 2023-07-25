#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/common/params.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

#define FONT_OPEN_SANS "Inter" //"Open Sans"
OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QStackedLayout *road_view_layout = new QStackedLayout;
  road_view_layout->setStackingMode(QStackedLayout::StackAll);
  nvg = new NvgWindow(VISION_STREAM_RGB_ROAD, this);
  road_view_layout->addWidget(nvg);
  hud = new OnroadHud(this);
  road_view_layout->addWidget(hud);

  nvg->hud = hud;
	
  buttons = new ButtonsWindow(this);
  stacked_layout->addWidget(buttons);


  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addLayout(road_view_layout);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);

  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);

  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignBottom);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();

}

void OnroadWindow::updateState(const UIState &s) {
  buttons->updateState(s);
	
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  alerts->updateAlert(alert);

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  hud->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
	
  UIState *my_s = uiState();
  if (s.scene.blinkerstatus || my_s->scene.prev_blinkerstatus) {
    update();
    my_s->scene.prev_blinkerstatus = s.scene.blinkerstatus;
    my_s->scene.blinkerframe += my_s->scene.blinkerframe < 255? +20 : -255;
  }	
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {
  QRect rc = rect();
  if(isMapVisible()) {
    UIState *s = uiState();
    if(!s->scene.map_on_left)
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
    else {
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
      rc.setX((topWidget(this)->width() / 2));
    }
  }
  if(rc.contains(e->pos())) {
#if 0
    QPoint endPos = e->pos();
    int dx = endPos.x() - startPos.x();
    int dy = endPos.y() - startPos.y();
    if(std::abs(dx) > 250 || std::abs(dy) > 200) {

      if(std::abs(dx) < std::abs(dy)) {

        if(dy < 0) { // upward
          Params().remove("CalibrationParams");
          Params().remove("LiveParameters");
          QTimer::singleShot(1500, []() {
            Params().putBool("SoftRestartTriggered", true);
          });

          QSound::play("../assets/sounds/reset_calibration.wav");
        }
        else { // downward
          QTimer::singleShot(500, []() {
            Params().putBool("SoftRestartTriggered", true);
          });
        }
      }
      else if(std::abs(dx) > std::abs(dy)) {
        if(dx < 0) { // right to left
          if(recorder)
            recorder->toggle();
        }
        else { // left to right
          if(recorder)
            recorder->toggle();
        }
      }

      return;
    }
#endif

    if (map != nullptr) {
      bool sidebarVisible = geometry().x() > 0;
      map->setVisible(!sidebarVisible && !map->isVisible());
    }
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {

  QRect rc = rect();
  if(isMapVisible()) {
    UIState *s = uiState();
    if(!s->scene.map_on_left)
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
    else {
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
      rc.setX((topWidget(this)->width() / 2));
    }
  }

  printf("%d, %d, %d, %d\n", rc.x(), rc.y(), rc.width(), rc.height());
  if(rc.contains(e->pos())) {
    startPos = e->pos();
  }

  QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->addWidget(m, 0, Qt::AlignRight);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

  alerts->updateAlert({});

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_RGB_WIDE_ROAD : VISION_STREAM_RGB_ROAD);

  if(offroad && recorder) {
    recorder->stop(false);
  }
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));

  // Begin AleSato Blinker Indicator
  p.setPen(Qt::NoPen);
  UIState *s = uiState();
  p.setBrush(QBrush(QColor(0, 0, 0, 0xff)));
  if (s->scene.blinkerstatus == 1) {
    // left rectangle for blinker indicator
    float rightcorner = width() * 0.75;
    QRect blackground = QRect(0, height()*0.75, rightcorner, height());
    p.drawRect(blackground);
    float bottomsect = rightcorner / (rightcorner + (height()/4)); // time proportion
    float delta = 1 - (float(s->scene.blinkerframe)/(255*bottomsect));
    delta = std::clamp(delta, 0.0f, 1.0f);
    QRect r = QRect(rightcorner*delta, height()-30, rightcorner-(rightcorner*delta), 30);
    p.setBrush(QBrush(QColor(255, 150, 0, 255)));
    p.drawRect(r);
    float delta2 = (float(s->scene.blinkerframe) - float(255 * bottomsect)) / (255 * (1 - bottomsect));
    delta2 = std::clamp(delta2, 0.0f, 1.0f);
    r = QRect(0, height() - height()*0.25*delta2, 30, height());
    p.drawRect(r);
  } else if (s->scene.blinkerstatus == 2) {
    // right rectangle for blinker indicator
    float leftcorner = width() * 0.25;
    QRect blackground = QRect(leftcorner, height()*0.75, width(), height());
    p.drawRect(blackground);
    float bottomsect = (width() - leftcorner) / (width() - leftcorner + (height()/4)); // time proportion
    float delta = float(s->scene.blinkerframe)/(255*bottomsect);
    delta = std::clamp(delta, 0.0f, 1.0f);
    QRect r = QRect(leftcorner, height()-30, (width()-leftcorner)*delta, 30);
    p.setBrush(QBrush(QColor(255, 150, 0, 150)));
    p.drawRect(r);
    float delta2 = (float(s->scene.blinkerframe) - float(255 * bottomsect)) / (255 * (1 - bottomsect));
    delta2 = std::clamp(delta2, 0.0f, 1.0f);
    r = QRect(width()-30, height() - height()*0.25*delta2, width(), height());
    p.drawRect(r);
  }
  // End AleSato Blinker Indicator	
}

// ***** onroad widgets *****

ButtonsWindow::ButtonsWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  QWidget *btns_wrapper = new QWidget;
  QHBoxLayout *btns_layout  = new QHBoxLayout(btns_wrapper);
  btns_layout->setSpacing(0);
  btns_layout->setContentsMargins(0, 770, 30, 30);
  main_layout->addWidget(btns_wrapper, 0, Qt::AlignTop);

  // Dynamic lane profile button
  QString initDlpBtn = "";
  dlpBtn = new QPushButton(initDlpBtn);
  QObject::connect(dlpBtn, &QPushButton::clicked, [=]() {
    uiState()->scene.dynamic_lane_profile = uiState()->scene.dynamic_lane_profile + 1;
    if (uiState()->scene.dynamic_lane_profile > 2) {
      uiState()->scene.dynamic_lane_profile = 0;
    }
    if (uiState()->scene.dynamic_lane_profile == 0) {
      Params().put("DynamicLaneProfile", "0", 1);
      dlpBtn->setText("Lane\nonly");
    } else if (uiState()->scene.dynamic_lane_profile == 1) {
      Params().put("DynamicLaneProfile", "1", 1);
      dlpBtn->setText("Lane\nless");
    } else if (uiState()->scene.dynamic_lane_profile == 2) {
      Params().put("DynamicLaneProfile", "2", 1);
      dlpBtn->setText("Auto\nLane");
    }
  });
  dlpBtn->setFixedWidth(186);
  dlpBtn->setFixedHeight(140);
  btns_layout->addWidget(dlpBtn, 0, Qt::AlignLeft);
  btns_layout->addSpacing(0);

  if (uiState()->scene.end_to_end) {
    dlpBtn->hide();
  }

  setStyleSheet(R"(
    QPushButton {
      color: white;
      text-align: center;
      padding: 0px;
      border-width: 9px;
      border-style: solid;
      background-color: rgba(0, 0, 0, 0.3);
    }
  )");
}

void ButtonsWindow::updateState(const UIState &s) {
  if (uiState()->scene.dynamic_lane_profile == 0) {
    dlpBtn->setStyleSheet(QString("font-size: 45px; border-radius: 100px; border-color: %1").arg(dlpBtnColors.at(0)));
    dlpBtn->setText("Lane\nonly");
  } else if (uiState()->scene.dynamic_lane_profile == 1) {
    dlpBtn->setStyleSheet(QString("font-size: 45px; border-radius: 100px; border-color: %1").arg(dlpBtnColors.at(1)));
    dlpBtn->setText("Lane\nless");
  } else if (uiState()->scene.dynamic_lane_profile == 2) {
    dlpBtn->setStyleSheet(QString("font-size: 45px; border-radius: 100px; border-color: %1").arg(dlpBtnColors.at(2)));
    dlpBtn->setText("Auto\nLane");
  }
}

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a) {
  if (!alert.equal(a)) {
    alert = a;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_heights = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_heights[alert.size];

  int margin = 40;
  int radius = 30;
  if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    margin = 0;
    radius = 0;
  }
  QRect r = QRect(0 + margin, height() - h + margin, width() - margin*2, h - margin*2);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(alert_colors[alert.status]));
  p.drawRoundedRect(r, radius, radius);	
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// OnroadHud
OnroadHud::OnroadHud(QWidget *parent) : QWidget(parent) {
  engage_img = QPixmap("../assets/img_chffr_wheel.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  //dm_img = QPixmap("../assets/img_driver_face.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  compass_inner_img = QPixmap("../assets/images/compass_inner.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  compass_outer_img = QPixmap("../assets/images/compass_outer.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  connect(this, &OnroadHud::valueChanged, [=] { update(); });
}

void OnroadHud::updateState(const UIState &s) {	
  const SubMaster &sm = *(s.sm);
  const auto cs = sm["controlsState"].getControlsState();
	
  setProperty("status", s.status);
  setProperty("ang_str", s.scene.angleSteers);
	
  // update engageability and DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    setProperty("engageable", cs.getEngageable() || cs.getEnabled());
    //setProperty("dmActive", sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode());
    setProperty("compass", s.scene.compass);
    setProperty("bearingDeg", sm["gpsLocationExternal"].getGpsLocationExternal().getBearingDeg());
    setProperty("bearingAccuracyDeg", sm["gpsLocationExternal"].getGpsLocationExternal().getBearingAccuracyDeg());  
  }
  if(uiState()->recording) {
    update();
  }
}

void OnroadHud::paintEvent(QPaintEvent *event) {
  //UIState *s = &QUIState::ui_state;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
	
  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);
	
  // engage-ability icon
  //if (engageable) {
  if (true) {
    drawIcon(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + bdr_s,
             engage_img, bg_colors[status], 5.0, true, ang_str );
  }
  // compass
  if (compass && bearingAccuracyDeg != 180.00) {
    drawCompass(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + bdr_s + 530,
                compass_outer_img, blackColor(180), 5.0, bearingDeg);
  }
}

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void OnroadHud::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, bool rotation, float angle) {
  // 
  if (rotation) {
    p.setPen(Qt::NoPen);
    p.setBrush(bg);
    p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
    p.setOpacity(opacity);
    p.save();
    p.translate(x, y);
    p.rotate(-angle);
    QRect r = img.rect();
    r.moveCenter(QPoint(0,0));
    p.drawPixmap(r, img);
    p.restore();
  } else {
    p.setPen(Qt::NoPen);
    p.setBrush(bg);
    p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
    p.setOpacity(opacity);
    p.drawPixmap(x - img_size / 2, y - img_size / 2, img);
  }
}

void OnroadHud::drawCompass(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, float bearing_Deg) {
  // Draw the circle background
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);

  // Rotate the compass_inner_img image
  p.save();
  p.translate(x, y);
  p.rotate(bearing_Deg);
  p.drawPixmap(-compass_inner_img.width() / 2, -compass_inner_img.height() / 2, compass_inner_img);
  p.restore();

  // Display compass_outer_img
  //QPixmap imgScaled = img.scaled(img.width() * 2, img.height() * 2, Qt::KeepAspectRatio);
  p.drawPixmap(x - img_size / 2, y - img_size / 2, img);

  // Set the font for the direction labels
  QFont font = p.font();
  font.setFamily("Inter");
  font.setBold(true);
  font.setPointSize(10);
  p.setFont(font);
  p.setPen(Qt::white);

  // Draw the cardinal directions
  const auto drawDirection = [&](const QString &text, float from, float to, int hAlign, int vAlign) {
    // Set the opacity based on whether the direction label is currently being pointed at
    p.setOpacity((bearing_Deg >= from && bearing_Deg < to) ? 1.0 : 0.2);
    p.drawText(x - radius / 2, y - radius / 2, radius, radius, hAlign | vAlign, text);
  };
  drawDirection("N", 0, 67.5, Qt::AlignTop | Qt::AlignHCenter, {});
  drawDirection("E", 22.5, 157.5, Qt::AlignRight | Qt::AlignVCenter, {});
  drawDirection("S", 112.5, 247.5, Qt::AlignBottom | Qt::AlignHCenter, {});
  drawDirection("W", 202.5, 337.5, Qt::AlignLeft | Qt::AlignVCenter, {});
  drawDirection("N", 292.5, 360, Qt::AlignTop | Qt::AlignHCenter, {});
}

// NvgWindow

NvgWindow::NvgWindow(VisionStreamType type, QWidget* parent) : last_update_params(0), fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraViewWidget("camerad", type, true, parent) {

}

void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);

  //neokii
  ic_brake = QPixmap("../assets/images/img_brake_disc.png");
  //ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  //ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  //ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");
  ic_scc2 = QPixmap("../assets/images/img_scc2.png");
  ic_radar = QPixmap("../assets/images/radar.png");
  ic_radar_vision = QPixmap("../assets/images/radar_vision.png");
  ic_lane_change_left_img = QPixmap("../assets/images/lane_change_left.png");
  ic_lane_change_right_img = QPixmap("../assets/images/lane_change_right.png");
	
}

void NvgWindow::updateFrameMat(int w, int h) {
  CameraViewWidget::updateFrameMat(w, h);

  UIState *s = uiState();
  s->fb_w = w;
  s->fb_h = h;
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }
  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2, h / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void NvgWindow::ui_draw_line(QPainter &painter, const line_vertices_data &vd) 
{
  if (vd.cnt == 0) return;
 
  QPainterPath path = QPainterPath();

  const QPointF *v = &vd.v[0];
  path.moveTo( v[0].x(), v[0].y() );
  for (int i = 1; i < vd.cnt; i++) {
    path.lineTo( v[i].x(), v[i].y());
  }
  painter.drawPath( path );
}

void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);
  int steerOverride = (*s->sm)["carState"].getCarState().getSteeringPressed();

  // paint blindspot line
  painter.setBrush(QColor(221, 160, 221, 200));

  if( scene.leftblindspot  )
  {
       ui_draw_line(  painter, scene.lane_blindspot_vertices[0] );
  }

  if( scene.rightblindspot  )
  {
   //  if( right_cnt > 1 )
        ui_draw_line( painter, scene.lane_blindspot_vertices[1] );
  }
	
  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    ui_draw_line( painter, scene.lane_line_vertices[i] );
  }
	
  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));

    ui_draw_line( painter, scene.road_edge_vertices[i] );
    //painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  }
	
  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration();
  float start_hue, end_hue, acceleration_future = 0;

  if (acceleration.getZ().size() > 16) {
    acceleration_future = acceleration.getX()[16];  // 2.5 seconds
  }
  start_hue = 60;
  // speed up: 120, slow down: 0
  end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);

  // FIXME: painter.drawPolygon can be slow if hue is not rounded
  end_hue = int(end_hue * 100 + 0.5) / 100;
	
  if ((*s->sm)["controlsState"].getControlsState().getEnabled()) {
  if (steerOverride) {
      bg.setColorAt(0.0, redColor(100));
      bg.setColorAt(0.5, redColor(50));  
      bg.setColorAt(1.0, redColor(0));
    } else {
      bg.setColorAt(0.0, scene.lateralPlan.dynamicLaneProfileStatus ? QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.35) : QColor::fromHslF(216 / 360., 0.94, 0.51, 0.35));
      bg.setColorAt(0.5, scene.lateralPlan.dynamicLaneProfileStatus ? QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.3) : QColor::fromHslF(190 / 360., 1.0, 0.68, 0.3));
      bg.setColorAt(1.0, scene.lateralPlan.dynamicLaneProfileStatus ? QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0) : QColor::fromHslF(190 / 360., 1.0, 0.68, 0.0));
    }
  } else {
    bg.setColorAt(0.0, whiteColor(100));
    bg.setColorAt(0.5, whiteColor(50));  
    bg.setColorAt(1.0, whiteColor(0));
  }
  painter.setBrush(bg);
  ui_draw_line( painter, scene.track_vertices );
	
  painter.restore();
}

static float global_a_rel;
static float global_a_rel_col;
static float vc_speed;
void NvgWindow::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd, int num ) {
  painter.save();
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }
	
  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());
	
  float g_xo = sz / 5;
  float g_yo = sz / 10;

  UIState *s = uiState();

  if (s->scene.radarDistance < 149) {
    float homebase_h = 12;
    QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo + homebase_h},{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo},{x - (sz * 1.35) - g_xo, y + sz + g_yo + homebase_h}, {x, y + sz + homebase_h + g_yo + 10}};
    painter.setBrush(QColor(218, 202, 37, 210));
    painter.drawPolygon(glow, std::size(glow));

    // chevron
    QPointF chevron[] = {{x + (sz * 1.25), y + sz + homebase_h},{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz},{x - (sz * 1.25), y + sz + homebase_h}, {x, y + sz + homebase_h - 7}};
    painter.setBrush(redColor(fillAlpha));
    painter.drawPolygon(chevron, std::size(chevron));
    configFont(painter, FONT_OPEN_SANS, 36, "ExtraBold");  
    painter.setPen(QColor(0x0, 0x0, 0xff));
    painter.drawText(QRect(x - (sz * 1.25), y, 2 * (sz * 1.25), sz * 1.25), Qt::AlignCenter, QString("R"));
  } else {
    float homebase_h = 12;  
    QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo + homebase_h},{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo},{x - (sz * 1.35) - g_xo, y + sz + g_yo + homebase_h}, {x, y + sz + homebase_h + g_yo + 10}};
    painter.setBrush(QColor(0, 255, 0, 255));
    painter.drawPolygon(glow, std::size(glow));

    // chevron
    QPointF chevron[] = {{x + (sz * 1.25), y + sz + homebase_h},{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz},{x - (sz * 1.25), y + sz + homebase_h}, {x, y + sz + homebase_h - 7}};
    painter.setBrush(greenColor(fillAlpha));
    painter.drawPolygon(chevron, std::size(chevron));
    configFont(painter, FONT_OPEN_SANS, 36, "ExtraBold");  
    painter.setPen(QColor(0x0, 0x0, 0x0));
    painter.drawText(QRect(x - (sz * 1.25), y, 2 * (sz * 1.25), sz * 1.25), Qt::AlignCenter, QString("V"));
  }	
  
  if(num == 0){
    QString dist = QString::number(d_rel,'f',0) + "m";
    int str_w = 200;
    //QString kmph = QString::number((v_rel + vc_speed)*3.6,'f',0) + "k";
    //int str_w2 = 200;

    configFont(painter, FONT_OPEN_SANS, 40, "SemiBold");
    painter.setPen(QColor(0x0, 0x0, 0x0 , 200));
    float lock_indicator_dx = 2;
    painter.drawText(QRect(x+2+lock_indicator_dx+90, y-50+65, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    //painter.drawText(QRect(x+2-lock_indicator_dx-str_w2-2, y-50+2, str_w2, 50), Qt::AlignBottom | Qt::AlignRight, kmph);
    painter.setPen(QColor(0xff, 0xff, 0xff));
    painter.drawText(QRect(x+lock_indicator_dx+90, y-50+65, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    if(global_a_rel >= global_a_rel_col){
      global_a_rel_col = -0.1;
      painter.setPen(QColor(0.09*255, 0.945*255, 0.26*255, 255));
    } else {
      global_a_rel_col = 0;
      painter.setPen(QColor(245, 0, 0, 255));
    }
    //painter.drawText(QRect(x-lock_indicator_dx-str_w2-2, y-50, str_w2, 50), Qt::AlignBottom | Qt::AlignRight, kmph);
    painter.setPen(Qt::NoPen);
  }

  painter.restore();
}

// Ichirio
struct LeadcarLockon {
  float x,y,d,a,lxt,lxf,lockOK;
};
#define LeadcarLockon_MAX 5
LeadcarLockon leadcar_lockon[LeadcarLockon_MAX];

void NvgWindow::drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num) {
  const float d_rel = lead_data.getX()[0];
  float a_rel = lead_data.getA()[0];
  global_a_rel = a_rel;

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = (float)vd.y();

  painter.setCompositionMode(QPainter::CompositionMode_Plus);

  float prob_alpha = lead_data.getProb();
  if(prob_alpha < 0){
    prob_alpha = 0;
  } else if(prob_alpha > 1.0){
    prob_alpha = 1.0;
  }
  prob_alpha *= 245;

  painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
  painter.setBrush(QColor(0, 0, 0, 0));
  float ww = 300 , hh = 300;
  if(Hardware::TICI()){
    ww *= 1.25; hh *= 1.25;
  }
  float d = d_rel;
  if(d < 1){
    d = 1;
  }

  leadcar_lockon[num].x = leadcar_lockon[num].x + (x - leadcar_lockon[num].x) / 6;
  leadcar_lockon[num].y = leadcar_lockon[num].y + (y - leadcar_lockon[num].y) / 6;
  leadcar_lockon[num].d = leadcar_lockon[num].d + (d - leadcar_lockon[num].d) / 6;
  x = leadcar_lockon[num].x;
  y = leadcar_lockon[num].y;
  d = leadcar_lockon[num].d;
  if(d < 1){
    d = 1;
  }

  leadcar_lockon[num].a = leadcar_lockon[num].a + (a_rel - leadcar_lockon[num].a) / 10;
  a_rel = leadcar_lockon[num].a;

  float dh = 50;
  ww *= 0.5; hh *= 0.5;
  dh = 100;
  float dd = d;
  dd -= 5;
  dd /= (95.0/10);
  dd += 1;
  if(dd < 1)dd = 1;
  dh /= dd*dd;
	
  ww = ww * 2 * 5 / d;
  hh = hh * 2 * 5 / d;
  y = std::fmin(height() /*- sz * .6*/, y - dh) + dh;
  QRect r = QRect(x - ww/2, y /*- g_yo*/ - hh - dh, ww, hh);

#if 0
  float y0 = lead0.getY()[0];
  float y1 = lead1.getY()[0];
#else
  float y0 = leadcar_lockon[0].x * leadcar_lockon[0].d;
  float y1 = leadcar_lockon[1].x * leadcar_lockon[1].d;
#endif

  configFont(painter, FONT_OPEN_SANS, 38, "SemiBold");
  if(num == 0){
    painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
    painter.drawRect(r);

    if(leadcar_lockon[0].x > leadcar_lockon[1].x - 20){
      leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.right() - leadcar_lockon[num].lxt) / 20;
      leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (width() - leadcar_lockon[num].lxf) / 20;
    } else {
      leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.left() - leadcar_lockon[num].lxt) / 20;
      leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (0 - leadcar_lockon[num].lxf) / 20;
    }
    painter.drawText(r, Qt::AlignTop | Qt::AlignLeft, " " + QString::number(num+1));

    float lxt = leadcar_lockon[num].lxt;
    if(lxt < r.left()){
      lxt = r.left();
    } else if(lxt > r.right()){
      lxt = r.right();
    }
    painter.drawLine(lxt,r.top() , leadcar_lockon[num].lxf , 0);
    if(ww >= 40){
      painter.setPen(Qt::NoPen);
      float wwa = ww * 0.15;
      if(wwa > 40){
        wwa = 40;
      } else if(wwa < 10){
        wwa = 10;
      }
      if(wwa > ww){
        wwa = ww;
      }

      float hha = 0;
      if(a_rel > 0){
        hha = 1 - 0.1 / a_rel;
        painter.setBrush(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha*0.9));

        if(hha < 0){
          hha = 0;
        }
        hha = hha * hh;
#if 0
        QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh + (hh-hha), wwa, hha);
        painter.drawRect(ra);
#else
        QPointF meter[] = {{(float)x + ww/2 - wwa/2 - wwa/2 * hha / hh , (float)y /*- g_yo*/ - hh - dh + (hh-hha)},{(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh + (hh-hha)}, {(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh + hh}, {(float)x + ww/2 - wwa/2 , (float)y /*- g_yo*/ - hh - dh + hh}};
        painter.drawPolygon(meter, std::size(meter));
#endif
      }
      if(a_rel < 0){
        hha = 1 + 0.1 / a_rel;
        painter.setBrush(QColor(245, 0, 0, prob_alpha));
        if(hha < 0){
          hha = 0;
        }
        hha = hha * hh;
#if 0
        QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh , wwa, hha);
        painter.drawRect(ra);
#else
        QPointF meter[] = {{(float)x + ww/2 - wwa/2 , (float)y /*- g_yo*/ - hh - dh},{(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh}, {(float)x + ww/2 , (float)y /*- g_yo*/ - hh - dh + hha}, {(float)x + ww/2 - wwa/2 - wwa/2 * hha / hh, (float)y /*- g_yo*/ - hh - dh + hha}};
        painter.drawPolygon(meter, std::size(meter));
#endif
      }
    }

    if(std::abs(y0 - y1) <= 300
    ){
      leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (40 - leadcar_lockon[num].lockOK) / 5;
    } else {
      leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (0 - leadcar_lockon[num].lockOK) / 5;
    }
    float td = leadcar_lockon[num].lockOK;
    if(td >= 3){
      float dd = leadcar_lockon[num].d;
      if(dd < 10){
        dd = 10;
      }
      dd -= 10;
      dd /= (90.0/2);
      dd += 1;
      td /= dd;

      float tlw = 8;
      float tlw_2 = tlw / 2;
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), tlw));
      painter.drawLine(r.center().x() , r.top()-tlw_2 , r.center().x() , r.top() - td);
      painter.drawLine(r.left()-tlw_2 , r.center().y() , r.left() - td , r.center().y());
      painter.drawLine(r.right()+tlw_2 , r.center().y() , r.right() + td , r.center().y());
      painter.drawLine(r.center().x() , r.bottom()+tlw_2 , r.center().x() , r.bottom() + td);
    }

  } else if(true){
    if(num == 1){
      if(std::abs(y0 - y1) > 300
      ){
        painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 2));
      } else {
        painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      }

      if(leadcar_lockon[0].x > leadcar_lockon[1].x - 20){
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.left() - leadcar_lockon[num].lxt) / 20;
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (0 - leadcar_lockon[num].lxf) / 20;
      } else {
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.right() - leadcar_lockon[num].lxt) / 20;
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (width() - leadcar_lockon[num].lxf) / 20;
      }
      float lxt = leadcar_lockon[num].lxt;
      if(lxt < r.left()){
        lxt = r.left();
      } else if(lxt > r.right()){
        lxt = r.right();
      }
      painter.drawLine(lxt,r.top() , leadcar_lockon[num].lxf , 0);

      if(ww >= 80){
      }
    } else if(num == 2){
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      //painter.drawLine(r.right(),r.center().y() , width() , height());
    } else {
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      //painter.drawLine(r.left(),r.center().y() , 0 , height());
    }

    painter.drawRect(r);

    if(ww >= 80){
      float d_lim = 12;
      if(num == 0 || (num==1 && (d_rel < d_lim || std::abs(y0 - y1) > 300))){
        painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(num+1));
      }
    }
    if(ww >= 160){
    }
  }
  painter.setPen(Qt::NoPen);
  painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
}

void NvgWindow::paintGL() {
  CameraViewWidget::paintGL();

  UIState *s = uiState();
  if (s->worldObjectsVisible()) { 
    if(!s->recording) {
      QPainter p(this);
      drawCommunity(p);
    }
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::NoPen);

    drawLaneLines(painter, s);
  }

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  auto now = millis_since_boot();
  if(now - last_update_params > 1000) {
    last_update_params = now;
    ui_update_params(uiState());
  }
  
  prev_draw_t = millis_since_boot();
}


void NvgWindow::drawCommunity(QPainter &p) {
  p.save();

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();
  const cereal::RadarState::Reader &radar_state = sm["radarState"].getRadarState();
	
  const auto leads = model.getLeadsV3();
  size_t leads_num = leads.size();
  for(size_t i=0; i<leads_num && i < LeadcarLockon_MAX; i++){
    if(leads[i].getProb() > .2){
      drawLockon(p, leads[i], s->scene.lead_vertices[i] , i);
    }
   auto lead_one = radar_state.getLeadOne();
   auto lead_two = radar_state.getLeadTwo();
   if (lead_one.getStatus()) {
     drawLead(p, lead_one, s->scene.lead_vertices[0], 0);
   }
   if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
     drawLead(p, lead_two, s->scene.lead_vertices[1], 1);
   }
  }
	
  drawMaxSpeed(p);
  drawSpeed(p);
  drawGpsStatus(p);
  drawBrake(p);
  drawMisc(p);
  drawLaneChangeIndicator(p, uiState());	
	
  if(s->show_steer)
    drawSteer(p);	
	
  if(s->show_engrpm)
    drawEngRpm(p);
	
  if(s->show_tpms && width() > 1200)
    drawTpms(p);
	
  if(s->show_debug && width() > 1200)
    drawDebugText(p);
	
  if(s->show_gear && width() > 1200)
    drawCgear(p);//기어
  	
  char str[1024];
  const auto car_state = sm["carState"].getCarState();
  const auto controls_state = sm["controlsState"].getControlsState();
  const auto car_params = sm["carParams"].getCarParams();
  const auto live_params = sm["liveParameters"].getLiveParameters();
  const auto device_state = sm["deviceState"].getDeviceState();	
  float distance_traveled = sm["controlsState"].getControlsState().getDistanceTraveled() / 1000;
  	
  int lateralControlState = controls_state.getLateralControlSelect();
  const char* lateral_state[] = {"PID", "INDI", "LQR", "TORQUE" };
	
  auto cpuList = device_state.getCpuTempC();
  float cpuTemp = 0;

  if (cpuList.size() > 0) {
      for(int i = 0; i < cpuList.size(); i++)
          cpuTemp += cpuList[i];
      cpuTemp /= cpuList.size();
  }

  auto cpu_loads = device_state.getCpuUsagePercent();
  int cpu_usage = std::accumulate(cpu_loads.begin(), cpu_loads.end(), 0) / cpu_loads.size();
	
  //int mdps_bus = car_params.getMdpsBus();
  int scc_bus = car_params.getSccBus();

  QString infoText;
  infoText.sprintf("         %s             SR %.2f             CPU온도 %.0f°C             CPU부하 %d%%             주행거리  %.1f km             SCC %d",
		      lateral_state[lateralControlState],
                      //live_params.getAngleOffsetDeg(),
                      //live_params.getAngleOffsetAverageDeg(),
                      controls_state.getSteerRatio(),
                      //controls_state.getSteerActuatorDelay(),
		      cpuTemp,
	              cpu_usage,
		      controls_state.getDistanceTraveled() / 1000,
                      scc_bus
                      );

  // info
  configFont(p, "Open Sans", 35, "Bold");
  p.setPen(QColor(0xff, 0xff, 0xff, 0xff));
  p.drawText(rect().left() + 180, rect().height() - 15, infoText);	
  const int h = 60;
  QRect bar_rc(rect().left(), rect().bottom() - h, rect().width(), h);
  p.setBrush(QColor(0, 0, 0, 20));
  p.drawRect(bar_rc);
  drawBottomIcons(p);
	
  p.setOpacity(1.);
}

void NvgWindow::drawSpeed(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getCluSpeedMs() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH));
  m_cur_speed = cur_speed;
  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color = QColor(255, 255, 255, 250);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 250);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 250);
  }

  QString speed;
  speed.sprintf("%.0f", cur_speed);
  configFont(p, "Open Sans", 176, "Bold");
  drawTextWithColor(p, rect().center().x(), 250, speed, color);

  configFont(p, "Open Sans", 66, "Regular");
  //drawText(p, rect().center().x(), 310, s->scene.is_metric ? "km/h" : "mph", 200)
}

static const QColor get_tpms_color(float tpms) {
    if(tpms < 5 || tpms > 60) // N/A
        return QColor(255, 255, 255, 220);
    if(tpms < 31)
        return QColor(255, 90, 90, 220);
    return QColor(255, 255, 255, 220);
}

static const QString get_tpms_text(float tpms) {
    if(tpms < 5 || tpms > 60)
        return "";

    char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return QString(str);
}

void NvgWindow::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

void NvgWindow::drawBottomIcons(QPainter &p) {
  UIState *s = uiState();	
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();
	
  int x = radius / 2 + (bdr_s * 2) + (radius + 50);
  const int y = rect().bottom() - footer_h / 2 - 10;

  // cruise gap
  int gap = car_state.getCruiseGap();
  bool longControl = scc_smoother.getLongControl();
  int autoTrGap = scc_smoother.getAutoTrGap();

  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(QColor(255, 255, 255, 255 * 0.0f)));
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);

  QString str;
  float textSize = 35.f;
  QColor textColor = QColor(255, 255, 255, 250);

  if(gap <= 0) {
    str = "N/A";
  }
  else if(longControl && gap == 1) {
    str = "SPORT";
    textColor = QColor(255, 255, 255, 250);
  }
  else if(longControl && gap == 2) {
    str = "NORMAL";
    textColor = QColor(255, 255, 255, 250);
  }
  else if(longControl && gap == 3) {
    str = "RELAX";
    textColor = QColor(255, 255, 255, 250);
  }
  else if(longControl && gap == 4) {
    str = "AUTO";
    textColor = QColor(255, 255, 255, 250);
  }
  else {
    str.sprintf("%d", (int)gap);
    textColor = QColor(255, 255, 225, 250);
    textSize = 35.f;
  }

  configFont(p, "Open Sans", 35, "Bold");
  drawText(p, x, y-30, "", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x-290, y+135, str, textColor);
	
 // Accel표시
  float accel = car_state.getAEgo();  
  float dx = 138 + 1330;
#ifdef __TEST  
  static float accel1 = 0.0;
  accel1 += 0.2;
  if (accel1 > 2.5) accel1 = -2.5;
  accel = accel1;
#endif	
  //QRect rectAccel(x + dx, y - 550, 35, 1100);
  //painter.setPen(Qt::NoPen);
  //p.setPen(QPen(Qt::white, 2));
  //p.setBrush(blackColor(150));
  //p.drawRect(rectAccel);
  QRect rectAccelPos(x + dx, y - 375, 35, -std::clamp((float)accel, -2.0f, 2.0f) / 2. * 550);
  p.setBrush((accel>=0.0)?greenColor(255):redColor(255));
  p.drawRect(rectAccelPos);
  //textColor = whiteColor(200);
  //configFont(p, "Inter", 25, "Bold");
  //drawTextWithColor(p, x +dx+20, y - 135, "", textColor);
	
 if (s->show_datetime && width() > 1200) {
     // ajouatom: 현재시간표시
     QTextOption  textOpt = QTextOption(Qt::AlignLeft);
     configFont(p, "Open Sans", 65, "Bold");
     p.drawText(QRect(270, 30, width(), 70), QDateTime::currentDateTime().toString("hh:mm"), textOpt);
     configFont(p, "Open Sans", 60, "Bold");
     p.drawText(QRect(270, 150, width(), 70), QDateTime::currentDateTime().toString("MM-dd(ddd)"), textOpt);
  }
	
  p.setOpacity(1.);
}

void NvgWindow::drawBrake(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  bool brake_valid = car_state.getBrakeLights();
	
  int w = 3200;
  int h = 70;
  int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
  int y = 40 - bdr_s + 960;
  
  if (brake_valid) {
    p.drawPixmap(x, y, w, h, ic_brake);
    p.setOpacity(1.f);
  }
}
	  
void NvgWindow::drawTpms(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(uiState()->sm);	
  auto car_state = sm["carState"].getCarState();

  const int w = 58;
  const int h = 126;
  const int x = 110 + 1610;
  const int y = height() - h - 68;

  auto tpms = car_state.getTpms();
  const float fl = tpms.getFl();
  const float fr = tpms.getFr();
  const float rl = tpms.getRl();
  const float rr = tpms.getRr();

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_tire_pressure);

  configFont(p, "Open Sans", 38, "Bold");

  QFontMetrics fm(p.font());
  QRect rcFont = fm.boundingRect("9");

  int center_x = x + 4;
  int center_y = y + h/2;
  const int marginX = (int)(rcFont.width() * 2.7f);
  const int marginY = (int)((h/2 - rcFont.height()) * 0.7f);

  drawText2(p, center_x-marginX-10, center_y-marginY-10-rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
  drawText2(p, center_x+marginX+10, center_y-marginY-10-rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
  drawText2(p, center_x-marginX-10, center_y+marginY+10, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
  drawText2(p, center_x+marginX+10, center_y+marginY+10, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));

  p.restore();
}

static QRect getRect(QPainter &p, int flags, QString text) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  return fm.boundingRect(init_rect, flags, text);
}

void NvgWindow::drawMaxSpeed(QPainter &p) {
  p.save();

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();
  const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  const auto car_params = sm["carParams"].getCarParams();

  bool is_metric = s->scene.is_metric;
  bool long_control = scc_smoother.getLongControl();

 // kph
  float applyMaxSpeed = scc_smoother.getApplyMaxSpeed();
  float cruiseMaxSpeed = scc_smoother.getCruiseMaxSpeed();

  bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);

  int activeNDA = road_limit_speed.getActive();
  int roadLimitSpeed = road_limit_speed.getRoadLimitSpeed();
  int camLimitSpeed = road_limit_speed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = road_limit_speed.getCamLimitSpeedLeftDist();
  int sectionLimitSpeed = road_limit_speed.getSectionLimitSpeed();
  int sectionLeftDist = road_limit_speed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  if(activeNDA > 0)
  {
      int w = 150;
      int h = 54;
      int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
      int y = 40 - bdr_s;

      p.setOpacity(1.f);
      p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
  }
  
  const int x_start = 30;
  const int y_start = 30;

  int board_width = 210;
  int board_height = 384;

  const int corner_radius = 32;
  int max_speed_height = 210;

  QColor bgColor = QColor(0, 0, 0, 166);

  {
    // draw board
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);

    if(limit_speed > 0 && left_dist > 0) {
      board_width = limit_speed < 100 ? 210 : 230;
      board_height = max_speed_height + board_width;

      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height-board_width/2), corner_radius, corner_radius);
      path.addRoundedRect(QRectF(x_start, y_start+corner_radius, board_width, board_height-corner_radius), board_width/2, board_width/2);
    }
    else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
      board_height = 485;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }
    else {
      max_speed_height = 235;
      board_height = max_speed_height;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }

    p.setPen(Qt::NoPen);
    p.fillPath(path.simplified(), bgColor);
  }
	
  QString str;
	
  // Max Speed
  {
    p.setPen(QColor(255, 255, 255, 230));
     
    if(is_cruise_set) {
      configFont(p, "Inter", 80, "Bold");

      if(is_metric)
        str.sprintf( "%d", (int)(cruiseMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(cruiseMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      configFont(p, "Inter", 60, "Bold");
      str = "N/A";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 35);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  } 

    
  // applyMaxSpeed
  {
    p.setPen(QColor(255, 255, 255, 180));

    configFont(p, "Inter", 50, "Bold");
    if(is_cruise_set && applyMaxSpeed > 0) {
      if(is_metric)
        str.sprintf( "%d", (int)(applyMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(applyMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      str = long_control ? "OP" : "MAX";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start + max_speed_height/2, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 24);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);  
  }
	
  //
  if(limit_speed > 0 && left_dist > 0) {
    QRect board_rect = QRect(x_start, y_start+board_height-board_width, board_width, board_width);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawEllipse(board_rect);

    padding = 18;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::red, 25));
    p.drawEllipse(board_rect);

    p.setPen(QPen(Qt::black, padding));

    str.sprintf("%d", limit_speed);
    configFont(p, "Inter", 70, "Bold");

    QRect text_rect = getRect(p, Qt::AlignCenter, str);
    QRect b_rect = board_rect;
    text_rect.moveCenter({b_rect.center().x(), 0});
    text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
    p.drawText(text_rect, Qt::AlignCenter, str);
	  
    // left dist
    QRect rcLeftDist;
    QString strLeftDist;

    if(left_dist < 1000)
      strLeftDist.sprintf("%dm", left_dist);
    else
      strLeftDist.sprintf("%.1fkm", left_dist / 1000.f);

    QFont font("Inter");
    font.setPixelSize(55);
    font.setStyleName("Bold");

    QFontMetrics fm(font);
    int width = fm.width(strLeftDist);

    padding = 10;

    int center_x = x_start + board_width / 2;
    rcLeftDist.setRect(center_x - width / 2, y_start+board_height+15, width, font.pixelSize()+10);
    rcLeftDist.adjust(-padding*2, -padding, padding*2, padding);

    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(rcLeftDist, 20, 20);

    configFont(p, "Inter", 55, "Bold");
    p.setBrush(Qt::NoBrush);
    p.setPen(QColor(255, 255, 255, 230));
    p.drawText(rcLeftDist, Qt::AlignCenter|Qt::AlignVCenter, strLeftDist);  
  }
  else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
    QRectF board_rect = QRectF(x_start, y_start+max_speed_height, board_width, board_height-max_speed_height);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawRoundedRect(board_rect, corner_radius-padding/2, corner_radius-padding/2);

    padding = 10;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::black, padding));
    p.drawRoundedRect(board_rect, corner_radius-12, corner_radius-12);

    {
      str = "SPEED\nLIMIT";
      configFont(p, "Inter", 35, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      str.sprintf("%d", roadLimitSpeed);
      configFont(p, "Inter", 75, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y()+board_rect.height()/2, board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 3);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      configFont(p, "Inter", 10, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    } 
  }

  p.restore();
}

void NvgWindow::drawMisc(QPainter &p) {
  p.save();
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);

  const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  QString currentRoadName = QString::fromStdString(road_limit_speed.getCurrentRoadName().cStr());

  QColor color = QColor(255, 255, 255, 230);

  configFont(p, "Inter", 70, "Regular");
  drawText(p, (width()-(bdr_s*2))/4 + bdr_s + 20, 140, currentRoadName, 200);

  p.restore();
}

void NvgWindow::drawSteer(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();

  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  configFont(p, "Open Sans", 50, "Bold");

  QString str;
  
  QRect rc(1660, 260, 184, 130);
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);
	
  QColor textColor0 = QColor(255, 255, 255, 200); // white
  QColor textColor1 = QColor(120, 255, 120, 200); // green
	
  str.sprintf("%.0f°", steer_angle);
  drawTextWithColor(p, rc.center().x(), rc.center().y(), str, textColor0);
	
  str.sprintf("%.0f°", desire_angle);
  drawTextWithColor(p, rc.center().x(), rc.center().y() + 50, str, textColor1);
}

void NvgWindow::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if(accuracy < 0.01f || accuracy > 20.f)
    return;

  int w = 150;
  int h = 62;
  int x = width() - w - 57;
  int y = 690;
  p.setOpacity(1.5);
  p.drawPixmap(x, y, w, h, ic_satellite);

  configFont(p, "Open Sans", 32, "Bold");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("GPS %.1f m", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);
  p.setOpacity(1.0);
}

void NvgWindow::drawDebugText(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  int y = 80;
  const int height = 60;

  const int text_x = width()/2 + 250;

  auto controls_state = sm["controlsState"].getControlsState();
  auto car_control = sm["carControl"].getCarControl();
  auto car_state = sm["carState"].getCarState();

  float applyAccel = controls_state.getApplyAccel();

  float aReqValue = controls_state.getAReqValue();
  float aReqValueMin = controls_state.getAReqValueMin();
  float aReqValueMax = controls_state.getAReqValueMax();

  //int sccStockCamAct = (int)controls_state.getSccStockCamAct();
  //int sccStockCamStatus = (int)controls_state.getSccStockCamStatus();

  float vEgo = car_state.getVEgo();
  float vEgoRaw = car_state.getVEgoRaw();
  int longControlState = (int)controls_state.getLongControlState();
  float vPid = controls_state.getVPid();
  float upAccelCmd = controls_state.getUpAccelCmd();
  float uiAccelCmd = controls_state.getUiAccelCmd();
  float ufAccelCmd = controls_state.getUfAccelCmd();
  float accel = car_control.getActuators().getAccel();

  const char* long_state[] = {"off", "pid", "stopping", "starting"};

  configFont(p, "Open Sans", 35, "Regular");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  str.sprintf("State: %s\n", long_state[longControlState]);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vEgo: %.2f/%.2f\n", vEgo*3.6f, vEgoRaw*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vPid: %.2f/%.2f\n", vPid, vPid*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("P: %.3f\n", upAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("I: %.3f\n", uiAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("F: %.3f\n", ufAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Accel: %.3f\n", accel);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Apply: %.3f, Stock: %.3f\n", applyAccel, aReqValue);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("%.3f (%.3f/%.3f)\n", aReqValue, aReqValueMin, aReqValueMax);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("aEgo: %.3f, %.3f\n", car_state.getAEgo(), car_state.getABasis());
  p.drawText(text_x, y, str);

  auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];

  float radar_dist = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getDRel() : 0;
  float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 1.5) : 0;

  y += height;
  str.sprintf("Lead: %.1f/%.1f/%.1f\n", radar_dist, vision_dist, (radar_dist - vision_dist));
  p.drawText(text_x, y, str);
}

void NvgWindow::drawCgear(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();

  auto t_gear = car_state.getCurrentGear();
  int shifter;

  shifter = int(car_state.getGearShifter());

  QString tgear, tgearshifter;

  tgear.sprintf("%.0f", t_gear);
  configFont(p, "Open Sans", 130, "Semi Bold");

  //shifter = 1;
	
  QRect rc(30, 620, 182, 135);
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);
	
  if ((t_gear < 9) && (t_gear !=0)) { 
    p.setPen(QColor(255, 255, 255, 255)); 
    p.drawText(rc.center().x() - 38, rc.center().y() + 48, tgear);
  } else if (t_gear == 14 ) { 
    p.setPen(QColor(201, 34, 49, 255));
    p.drawText(rc.center().x() - 38, rc.center().y() + 48, "R");
  } else if (shifter == 1 ) { 
    p.setPen(QColor(255, 255, 255, 255));
    p.drawText(rc.center().x() - 38, rc.center().y() + 48, "P");
  } else if (shifter == 3 ) {  
    p.setPen(QColor(255, 255, 255, 255));
    p.drawText(rc.center().x() - 40, rc.center().y() + 48, "N");
  }
}

void NvgWindow::drawEngRpm(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();

  float eng_rpm = car_state.getEngRpm();
  float textSize = 50;
	
  int x = (width() + (bdr_s*2))/2 - bdr_s;
  int y = bdr_s + 290;

  QString rpm;

  rpm.sprintf("%.0f", eng_rpm);
  configFont(p, "Open Sans", textSize, "Bold");

  QColor textColor0 = QColor(255, 255, 255, 250);
  QColor textColor1 = QColor(120, 255, 120, 250);
  QColor textColor2 = QColor(255, 255, 0, 250);
  QColor textColor3 = QColor(255, 0, 0, 250);

  if (eng_rpm < 1099) {
   drawTextWithColor(p, x, y, rpm, textColor0);
  } else if (eng_rpm < 2300) {
   drawTextWithColor(p, x, y, rpm, textColor1);
  } else if (eng_rpm < 2999) {
   drawTextWithColor(p, x, y, rpm, textColor2);
  } else if (eng_rpm > 3000) {
   drawTextWithColor(p, x, y, rpm, textColor2);
  }
}

void NvgWindow::drawLaneChangeIndicator(QPainter &painter, const UIState *s) {
  typedef cereal::LateralPlan::LaneChangeDirection Direction;
  typedef cereal::LateralPlan::LaneChangeState State;

  auto draw_indicator_lambda = [this](QPainter &painter, Direction direction, QColor color) {
    QPixmap img = direction == Direction::LEFT ? ic_lane_change_left_img : ic_lane_change_right_img;
    QRect img_rc{0, (rect().height() - img.height()) / 2, img.width() + 20, img.height() + 20};
    //QRect ellipse_rc = img_rc.adjusted(-img_rc.width(), -img_rc.height() / 2, 20, img_rc.height() / 2);
    if (direction == Direction::RIGHT) {
      img_rc.moveRight(rect().right() -400);
      //ellipse_rc.moveRight(rect().right() + img_rc.width() - 400);
    } else if (direction == Direction::LEFT) {
      img_rc.moveLeft(rect().left() + 400);
      //ellipse_rc.moveLeft(rect().left() + img_rc.width() + 400);
    }
    painter.setPen(Qt::NoPen);
    painter.setBrush(color);
    //painter.drawEllipse(ellipse_rc);
    painter.drawPixmap(img_rc, img);
  };

  auto lateralPlan = (*(s->sm))["lateralPlan"].getLateralPlan();
  auto laneChangeState = lateralPlan.getLaneChangeState();
  auto direction = lateralPlan.getLaneChangeDirection();

  if (laneChangeState == State::PRE_LANE_CHANGE) {
    auto carState = (*(s->sm))["carState"].getCarState();
    bool blocked = (direction == Direction::LEFT && carState.getLeftBlindspot()) ||
                   (direction == Direction::RIGHT && carState.getRightBlindspot());
    draw_indicator_lambda(painter, direction, blocked ? redColor(200) : blackColor(200));
  } else if (laneChangeState == State::LANE_CHANGE_STARTING ||
             laneChangeState == State::LANE_CHANGE_FINISHING) {
    draw_indicator_lambda(painter, direction, bg_colors[s->status]);
  }
}
