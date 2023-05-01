#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>
#include <QSound>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/common/params.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

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
  QObject::connect(uiState(), &UIState::uiUpdate, buttons, &ButtonsWindow::updateState);
  QObject::connect(nvg, &NvgWindow::resizeSignal, [=] (int w) {
    buttons->setFixedWidth(w);
  });
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

#ifdef QCOM2
  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);
  /*
  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignCenter);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();*/
#endif
}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    } else if (alert.type == "controlsUnresponsivePermanent") {
      bgColor = bg_colors[STATUS_DISENGAGED];
    }
    alerts->updateAlert(alert, bgColor);
  }

  hud->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {

#ifdef QCOM2
  // neokii
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

  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
#endif  
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
#ifdef QCOM2
  startPos = e->pos();
#else
  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
#endif 
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

  alerts->updateAlert({}, bg);

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_RGB_WIDE_ROAD : VISION_STREAM_RGB_ROAD);

#ifdef QCOM2
  if(offroad && recorder) {
    recorder->stop(false);
  }
#endif 
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
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
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
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
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_turn_signal_l = QPixmap("../assets/images/turn_signal_l.png");
  ic_turn_signal_r = QPixmap("../assets/images/turn_signal_r.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");
  ic_scc2 = QPixmap("../assets/images/img_scc2.png");
  ic_radar = QPixmap("../assets/images/radar.png");
  ic_radar_vision = QPixmap("../assets/images/radar_vision.png");
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

void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool is_radar) {
  painter.save();
  
  UIState* s = uiState();
  SubMaster& sm = *(s->sm);
  auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];
  bool radar_detected = lead_radar.getStatus() && lead_radar.getRadar();
	
  const int icon_size = 256;
  const float d_rel = lead_data.getX()[0];
  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  int circle_size = 160;
  QColor bgColor = QColor(0, 0, 0, 166);
  float radar_dist = radar_detected ? lead_radar.getDRel() : 0;
  float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 0) : 0;
  float disp_dist = (radar_detected) ? radar_dist : vision_dist;

#ifdef __TEST
  radar_detected = true;
  disp_dist = 127.0;
#endif

  QString str;
  QColor textColor = QColor(255, 255, 255, 255);

  if (radar_detected) {
      float radar_rel_speed = lead_radar.getVRel();
      //str.sprintf("%.0fkm/h", m_cur_speed + radar_rel_speed * 3.6);
      if (disp_dist < 21) bgColor = redColor(180);
      else if (disp_dist < 51) bgColor = orangeColor(180);
      else if (disp_dist < 81) bgColor = greenColor(200);  
      else if (disp_dist > 80)bgColor = blackColor(200);

      painter.setPen(Qt::NoPen);
      painter.setBrush(bgColor);
      painter.drawEllipse(x - circle_size / 2, y - circle_size / 2, circle_size, circle_size);
  }
  painter.setOpacity(1.0);
  painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, (radar_detected) ? ic_radar : ic_radar_vision);
  configFont(painter, "Inter", 60, "Bold");
  if(disp_dist<10.0) str.sprintf("%.1f", disp_dist);
  else str.sprintf("%.0f", disp_dist);
  drawTextWithColor(painter, x, y + 22.0, str, textColor);
	
	
  painter.restore();
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

  const SubMaster &sm = *(s->sm);

  auto leads = sm["modelV2"].getModelV2().getLeadsV3();
  if (leads[0].getProb() > .5) {
    drawLead(p, leads[0], s->scene.lead_vertices[0], s->scene.lead_radar[0]);
  }
  if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
    drawLead(p, leads[1], s->scene.lead_vertices[1], s->scene.lead_radar[1]);
  }

  drawMaxSpeed(p);
  drawSpeed(p);
  drawTurnSignals(p);
  drawGpsStatus(p);
  drawBrake(p);
	
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
  //const auto car_params = sm["carParams"].getCarParams();
  const auto live_params = sm["liveParameters"].getLiveParameters();
  const auto device_state = sm["deviceState"].getDeviceState();
	
  int lateralControlState = controls_state.getLateralControlSelect();
  const char* lateral_state[] = {"PID", "INDI", "LQR", "TORQUE" };
	
  auto cpuList = device_state.getCpuTempC();
  float cpuTemp = 0;

  if (cpuList.size() > 0) {
      for(int i = 0; i < cpuList.size(); i++)
          cpuTemp += cpuList[i];
      cpuTemp /= cpuList.size();
  }

  //int mdps_bus = car_params.getMdpsBus();
  //int scc_bus = (int)car_params.getSccBus();

  QString infoText;
  infoText.sprintf("    %s             SR%.2f             CPU %.1f°",
		      lateral_state[lateralControlState],
                      //live_params.getAngleOffsetDeg(),
                      //live_params.getAngleOffsetAverageDeg(),
                      controls_state.getSteerRatio(),
                      //controls_state.getSteerActuatorDelay(),
		      cpuTemp
                      //scc_bus
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
      configFont(p, "Open Sans", 36, "Bold");
      p.drawText(QRect(1158, 985, width(), 150), QDateTime::currentDateTime().toString("MM월dd ddd hh:mm"), textOpt);
      //configFont(p, "Open Sans", 60, "Bold");
      //p.drawText(QRect(270, 150, width(), 100), QDateTime::currentDateTime().toString("MM월 dd일 (ddd)"), textOpt);	
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
  const int x = 110 + 1617;
  const int y = height() - h - 90;

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

  drawText2(p, center_x-marginX, center_y-marginY-rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
  drawText2(p, center_x+marginX, center_y-marginY-rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
  drawText2(p, center_x-marginX, center_y+marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
  drawText2(p, center_x+marginX, center_y+marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));

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

  int sccBus = (int)car_params.getSccBus();
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
  if (sccBus == 2) 
  {
      int w = 150;
      int h = 54;
      int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s + 165;
      int y = 40 - bdr_s;  
      p.drawPixmap(x, y, w, h, ic_scc2); 
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
  }

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

void NvgWindow::drawTurnSignals(QPainter &p) {
  static int blink_index = 0;
  static int blink_wait = 0;
  static double prev_ts = 0.0;

  if(blink_wait > 0) {
    blink_wait--;
    blink_index = 0;
  }
  else {
    const SubMaster &sm = *(uiState()->sm);
    auto car_state = sm["carState"].getCarState();
    bool left_on = car_state.getLeftBlinker();
    bool right_on = car_state.getRightBlinker();

    const float img_alpha = 0.8f;
    const int fb_w = width() / 2 - 200;
    const int center_x = width() / 2;
    const int w = fb_w / 25;
    const int h = 170;
    const int gap = fb_w / 25;
    const int margin = (int)(fb_w / 3.8f);
    const int base_y = (height() - h) / 2 - 360;
    const int draw_count = 7;

    int x = center_x;
    int y = base_y;

    if(left_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        p.setOpacity(alpha);
        float factor = (float)draw_count / (i + draw_count);
        p.drawPixmap(x - w - margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_l);
        x -= gap + w;
      }
    }

    x = center_x;
    if(right_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        float factor = (float)draw_count / (i + draw_count);
        p.setOpacity(alpha);
        p.drawPixmap(x + margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_r);
        x += gap + w;
      }
    }

    if(left_on || right_on) {

      double now = millis_since_boot();
      if(now - prev_ts > 900/UI_FREQ) {
        prev_ts = now;
        blink_index++;
      }

      if(blink_index >= draw_count) {
        blink_index = draw_count - 1;
        blink_wait = UI_FREQ/4;
      }
    }
    else {
      blink_index = 0;
    }
  }

  p.setOpacity(1.);
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
  float textSize = 35;
	
  int x = (width() + (bdr_s*2))/2 - bdr_s + 20;
  int y = bdr_s + 290 + 717;

  QString rpm;

  rpm.sprintf("RPM %.0f", eng_rpm);
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
