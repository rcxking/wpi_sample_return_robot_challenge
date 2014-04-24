/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rtabmap/gui/ImageView.h"

#include <QtGui/QWheelEvent>
#include <QtCore/qmath.h>
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtCore/QDir>
#include <QtGui/QAction>
#include <QtGui/QGraphicsEffect>
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/gui/KeypointItem.h"

namespace rtabmap {

ImageView::ImageView(QWidget * parent) :
		QGraphicsView(parent),
		_zoom(250),
		_minZoom(250),
		_savedFileName((QDir::homePath()+ "/") + "picture" + ".png"),
		_image(0),
		_imageDepth(0)
{
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	this->setScene(new QGraphicsScene(this));
	connect(this->scene(), SIGNAL(sceneRectChanged(const QRectF &)), this, SLOT(updateZoom()));

	_menu = new QMenu(tr(""), this);
	_showImage = _menu->addAction(tr("Show image"));
	_showImage->setCheckable(true);
	_showImage->setChecked(true);
	_showImageDepth = _menu->addAction(tr("Show image depth"));
	_showImageDepth->setCheckable(true);
	_showImageDepth->setChecked(false);
	_showFeatures = _menu->addAction(tr("Show features"));
	_showFeatures->setCheckable(true);
	_showFeatures->setChecked(true);
	_showLines = _menu->addAction(tr("Show lines"));
	_showLines->setCheckable(true);
	_showLines->setChecked(true);
	_saveImage = _menu->addAction(tr("Save picture..."));
}

ImageView::~ImageView() {
	clear();
}

void ImageView::resetZoom()
{
	_zoom = _minZoom;
	this->setDragMode(QGraphicsView::NoDrag);
}

bool ImageView::isImageShown()
{
	return _showImage->isChecked();
}

bool ImageView::isImageDepthShown()
{
	return _showImageDepth->isChecked();
}

bool ImageView::isFeaturesShown()
{
	return _showFeatures->isChecked();
}

void ImageView::setFeaturesShown(bool shown)
{
	_showFeatures->setChecked(shown);
	for(int i=0; i<_features.size(); ++i)
	{
		_features[i]->setVisible(_showFeatures->isChecked());
	}
}

void ImageView::setImageShown(bool shown)
{
	_showImage->setChecked(shown);
	if(_image)
	{
		_image->setVisible(_showImage->isChecked());
		this->updateOpacity();
	}
}

void ImageView::setImageDepthShown(bool shown)
{
	_showImageDepth->setChecked(shown);
	if(_imageDepth)
	{
		_imageDepth->setVisible(_showImageDepth->isChecked());
		this->updateOpacity();
	}
}

bool ImageView::isLinesShown()
{
	return _showLines->isChecked();
}

void ImageView::setLinesShown(bool shown)
{
	_showLines->setChecked(shown);
	QList<QGraphicsItem*> items = this->scene()->items();
	for(int i=0; i<items.size(); ++i)
	{
		if( qgraphicsitem_cast<QGraphicsLineItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_showLines->isChecked());
		}
	}
}

void ImageView::contextMenuEvent(QContextMenuEvent * e)
{
	QAction * action = _menu->exec(e->globalPos());
	if(action == _saveImage)
	{
		QString text;
#ifdef QT_SVG_LIB
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), _savedFileName, "*.png *.xpm *.jpg *.pdf *.svg");
#else
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), _savedFileName, "*.png *.xpm *.jpg *.pdf");
#endif
		if(!text.isEmpty())
		{
			_savedFileName = text;
			QImage img(this->sceneRect().width(), this->sceneRect().height(),QImage::Format_ARGB32_Premultiplied);
			QPainter p(&img);
			this->scene()->render(&p, this->sceneRect(), this->sceneRect());
			img.save(text);
		}
	}
	else if(action == _showFeatures)
	{
		this->setFeaturesShown(_showFeatures->isChecked());
	}
	else if(action == _showImage)
	{
		this->setImageShown(_showImage->isChecked());
	}
	else if(action == _showImageDepth)
	{
		this->setImageDepthShown(_showImageDepth->isChecked());
	}
	else if(action == _showLines)
	{
		this->setLinesShown(_showLines->isChecked());
	}

	if(action == _showImage || action ==_showImageDepth)
	{
		this->updateOpacity();
	}
}

void ImageView::updateOpacity()
{
	if(_image && _imageDepth)
	{
		if(_image->isVisible() && _imageDepth->isVisible())
		{
			QGraphicsOpacityEffect * effect = new QGraphicsOpacityEffect();
			QGraphicsOpacityEffect * effect2 = new QGraphicsOpacityEffect();
			effect->setOpacity(0.5);
			effect2->setOpacity(0.5);
			_image->setGraphicsEffect(effect);
			_imageDepth->setGraphicsEffect(effect2);
		}
		else
		{
			_image->setGraphicsEffect(0);
			_imageDepth->setGraphicsEffect(0);
		}
	}
}

void ImageView::updateZoom()
{
	qreal scaleRatio = 1;
	if(this->scene())
	{
		scaleRatio = this->geometry().width()/this->sceneRect().width();
	}
	_minZoom = log(scaleRatio)/log(2.0)*50+250;
}

void ImageView::wheelEvent(QWheelEvent * e)
{
	if(e->delta() > 0)
	{
		_zoom += 20;
		this->setDragMode(QGraphicsView::ScrollHandDrag);
		if(_zoom>=500)
		{
			_zoom = 500;
		}
	}
	else
	{
		_zoom -= 20;
		if(_zoom<=_minZoom)
		{
			this->setDragMode(QGraphicsView::NoDrag);
			_zoom = _minZoom;
			this->fitInView(this->sceneRect(), Qt::KeepAspectRatio);
			return;
		}
	}

	qreal scale = qPow(qreal(2), (_zoom - 250) / qreal(50));
	QMatrix matrix;
	matrix.scale(scale, scale);
	this->setMatrix(matrix);
}

void ImageView::setFeatures(const std::multimap<int, cv::KeyPoint> & refWords)
{
	for(int i=0; i<_features.size(); ++i)
	{
		scene()->removeItem(_features[i]);
		delete _features[i];
	}
	_features.clear();

	rtabmap::KeypointItem * item = 0;
	int alpha = 70;
	for(std::multimap<int, cv::KeyPoint>::const_iterator i = refWords.begin(); i != refWords.end(); ++i )
	{
		const cv::KeyPoint & r = (*i).second;
		int id = (*i).first;
		QString info = QString( "WordRef = %1\n"
								"Laplacian = %2\n"
								"Dir = %3\n"
								"Hessian = %4\n"
								"X = %5\n"
								"Y = %6\n"
								"Size = %7").arg(id).arg(1).arg(r.angle).arg(r.response).arg(r.pt.x).arg(r.pt.y).arg(r.size);
		float radius = r.size*1.2/9.*2;

		item = new rtabmap::KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 255, 0, alpha));

		scene()->addItem(item);
		_features.append(item);
		item->setVisible(_showFeatures->isChecked());
		item->setZValue(1);
	}
}

void ImageView::setImage(const QImage & image)
{
	if(_image)
	{
		_image->setPixmap(QPixmap::fromImage(image));
	}
	else
	{
		_image = scene()->addPixmap(QPixmap::fromImage(image));
		_image->setVisible(_showImage->isChecked());
		_showImage->setEnabled(true);
		this->updateOpacity();
	}
}

void ImageView::setImageDepth(const QImage & imageDepth)
{
	if(_imageDepth)
	{
		_imageDepth->setPixmap(QPixmap::fromImage(imageDepth));
	}
	else
	{
		_imageDepth = scene()->addPixmap(QPixmap::fromImage(imageDepth));
		_imageDepth->setVisible(_showImageDepth->isChecked());
		_showImageDepth->setEnabled(true);
		this->updateOpacity();
	}
}

void ImageView::clear()
{
	for(int i=0; i<_features.size(); ++i)
	{
		scene()->removeItem(_features[i]);
		delete _features[i];
	}
	_features.clear();

	if(_image)
	{
		scene()->removeItem(_image);
		delete _image;
		_image = 0;
		_showImage->setEnabled(false);
	}

	if(_imageDepth)
	{
		scene()->removeItem(_imageDepth);
		delete _imageDepth;
		_imageDepth = 0;
		_showImageDepth->setEnabled(false);
	}
	scene()->clear();
}

}
