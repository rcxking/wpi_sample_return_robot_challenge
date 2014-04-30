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

#include "BayesFilter.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Parameters.h"
#include <iostream>

#include "rtabmap/utilite/UtiLite.h"

namespace rtabmap {

BayesFilter::BayesFilter(const ParametersMap & parameters) :
	_virtualPlacePrior(Parameters::defaultBayesVirtualPlacePriorThr()),
	_fullPredictionUpdate(Parameters::defaultBayesFullPredictionUpdate()),
	_totalPredictionLCValues(0.0f)
{
	this->setPredictionLC(Parameters::defaultBayesPredictionLC());
	this->parseParameters(parameters);
}

BayesFilter::~BayesFilter() {
}

void BayesFilter::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kBayesPredictionLC())) != parameters.end())
	{
		this->setPredictionLC((*iter).second);
	}
	Parameters::parse(parameters, Parameters::kBayesVirtualPlacePriorThr(), _virtualPlacePrior);
	Parameters::parse(parameters, Parameters::kBayesFullPredictionUpdate(), _fullPredictionUpdate);

	UASSERT(_virtualPlacePrior >= 0 && _virtualPlacePrior <= 1.0f);
}

// format = {Virtual place, Loop closure, level1, level2, l3, l4...}
void BayesFilter::setPredictionLC(const std::string & prediction)
{
	std::list<std::string> strValues = uSplit(prediction, ' ');
	if(strValues.size() < 2)
	{
		UERROR("The number of values < 2 (prediction=\"%s\")", prediction.c_str());
	}
	else
	{
		std::vector<double> tmpValues(strValues.size());
		int i=0;
		bool valid = true;
		for(std::list<std::string>::iterator iter = strValues.begin(); iter!=strValues.end(); ++iter)
		{
			tmpValues[i] = std::atof((*iter).c_str());
			//UINFO("%d=%e", i, tmpValues[i]);
			if(tmpValues[i] < 0.0 || tmpValues[i]>1.0)
			{
				valid = false;
				break;
			}
			++i;
		}

		if(!valid)
		{
			UERROR("The prediction is not valid (values must be between >0 && <=1) prediction=\"%s\"", prediction.c_str());
		}
		else
		{
			_predictionLC = tmpValues;
		}
	}
	_totalPredictionLCValues = 0.0f;
	for(unsigned int j=0; j<_predictionLC.size(); ++j)
	{
		_totalPredictionLCValues += _predictionLC[j];
	}
}

const std::vector<double> & BayesFilter::getPredictionLC() const
{
	// {Vp, Lc, l1, l2, l3, l4...}
	return _predictionLC;
}

std::string BayesFilter::getPredictionLCStr() const
{
	std::string values;
	for(unsigned int i=0; i<_predictionLC.size(); ++i)
	{
		values.append(uNumber2Str(_predictionLC[i]));
		if(i+1 < _predictionLC.size())
		{
			values.append(" ");
		}
	}
	return values;
}

void BayesFilter::reset()
{
	_posterior.clear();
	_prediction = cv::Mat();
}

const std::map<int, float> & BayesFilter::computePosterior(const Memory * memory, const std::map<int, float> & likelihood)
{
	ULOGGER_DEBUG("");

	if(!memory)
	{
		ULOGGER_ERROR("Memory is Null!");
		return _posterior;
	}

	if(!likelihood.size())
	{
		ULOGGER_ERROR("likelihood is empty!");
		return _posterior;
	}

	if(_predictionLC.size() < 2)
	{
		ULOGGER_ERROR("Prediction is not valid!");
		return _posterior;
	}

	UTimer timer;
	timer.start();

	cv::Mat prior;
	cv::Mat posterior;

	float sum = 0;
	int j=0;
	// Recursive Bayes estimation...
	// STEP 1 - Prediction : Prior*lastPosterior
	_prediction = this->generatePrediction(memory, uKeys(likelihood));

	ULOGGER_DEBUG("STEP1-generate prior=%fs, rows=%d, cols=%d", timer.ticks(), _prediction.rows, _prediction.cols);
	//std::cout << "Prediction=" << _prediction << std::endl;

	// Adjust the last posterior if some images were
	// reactivated or removed from the working memory
	posterior = cv::Mat(likelihood.size(), 1, CV_32FC1);
	this->updatePosterior(memory, uKeys(likelihood));
	j=0;
	for(std::map<int, float>::const_iterator i=_posterior.begin(); i!= _posterior.end(); ++i)
	{
		((float*)posterior.data)[j++] = (*i).second;
	}
	ULOGGER_DEBUG("STEP1-update posterior=%fs, posterior=%d, _posterior size=%d", posterior.rows, _posterior.size());
	//std::cout << "LastPosterior=" << posterior << std::endl;

	// Multiply prediction matrix with the last posterior
	// (m,m) X (m,1) = (m,1)
	prior = _prediction * posterior;
	ULOGGER_DEBUG("STEP1-matrix mult time=%fs", timer.ticks());
	//std::cout << "ResultingPrior=" << prior << std::endl;

	ULOGGER_DEBUG("STEP1-matrix mult time=%fs", timer.ticks());
	std::vector<float> likelihoodValues = uValues(likelihood);
	//std::cout << "Likelihood=" << cv::Mat(likelihoodValues) << std::endl;

	// STEP 2 - Update : Multiply with observations (likelihood)
	j=0;
	for(std::map<int, float>::const_iterator i=likelihood.begin(); i!= likelihood.end(); ++i)
	{
		std::map<int, float>::iterator p =_posterior.find((*i).first);
		if(p!= _posterior.end())
		{
			(*p).second = (*i).second * ((float*)prior.data)[j++];
			sum+=(*p).second;
		}
		else
		{
			ULOGGER_ERROR("Problem1! can't find id=%d", (*i).first);
		}
	}
	ULOGGER_DEBUG("STEP2-likelihood time=%fs", timer.ticks());
	//std::cout << "Posterior (before normalization)=" << _posterior << std::endl;

	// Normalize
	ULOGGER_DEBUG("sum=%f", sum);
	if(sum != 0)
	{
		for(std::map<int, float>::iterator i=_posterior.begin(); i!= _posterior.end(); ++i)
		{
			(*i).second /= sum;
		}
	}
	ULOGGER_DEBUG("normalize time=%fs", timer.ticks());
	//std::cout << "Posterior=" << _posterior << std::endl;

	return _posterior;
}

cv::Mat BayesFilter::generatePrediction(const Memory * memory, const std::vector<int> & ids) const
{
	if(!_fullPredictionUpdate && !_prediction.empty())
	{
		return updatePrediction(_prediction, memory, uKeys(_posterior), ids);
	}
	UDEBUG("");

	UASSERT(memory &&
		   _predictionLC.size() >= 2 &&
		   ids.size());

	UTimer timer;
	timer.start();
	UTimer timerGlobal;
	timerGlobal.start();

	std::map<int, int> idToIndexMap;
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		UASSERT_MSG(ids[i] != 0, "Signature id is null ?!?");
		idToIndexMap.insert(idToIndexMap.end(), std::make_pair(ids[i], i));
	}

	//int rows = prediction.rows;
	cv::Mat prediction = cv::Mat::zeros(ids.size(), ids.size(), CV_32FC1);
	int cols = prediction.cols;

	// Each prior is a column vector
	UDEBUG("_predictionLC.size()=%d",_predictionLC.size());
	std::set<int> idsDone;

	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(idsDone.find(ids[i]) == idsDone.end())
		{
			if(ids[i] > 0)
			{
				// Set high values (gaussians curves) to loop closure neighbors

				// ADD prob for each neighbors
				std::map<int, int> neighbors = memory->getNeighborsId(ids[i], _predictionLC.size()-1, 0);
				std::list<int> idsLoopMargin;
				//filter neighbors in STM
				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
				{
					if(memory->isInSTM(iter->first))
					{
						neighbors.erase(iter++);
					}
					else
					{
						if(iter->second == 0)
						{
							idsLoopMargin.push_back(iter->second);
						}
						++iter;
					}
				}

				// should at least have 1 id in idsMarginLoop
				if(idsLoopMargin.size() == 0)
				{
					UFATAL("No 0 margin neighbor for signature %d !?!?", ids[i]);
				}

				// same neighbor tree for loop signatures (margin = 0)
				for(std::list<int>::iterator iter = idsLoopMargin.begin(); iter!=idsLoopMargin.end(); ++iter)
				{
					float sum = 0.0f; // sum values added
					sum += this->addNeighborProb(prediction, i, neighbors, idToIndexMap);
					idsDone.insert(*iter);
					this->normalize(prediction, i, sum, ids[0]<0);
				}
			}
			else
			{
				// Set the virtual place prior
				if(_virtualPlacePrior > 0)
				{
					if(cols>1) // The first must be the virtual place
					{
						((float*)prediction.data)[i] = _virtualPlacePrior;
						float val = (1.0-_virtualPlacePrior)/(cols-1);
						for(int j=1; j<cols; j++)
						{
							((float*)prediction.data)[i + j*cols] = val;
						}
					}
					else if(cols>0)
					{
						((float*)prediction.data)[i] = 1;
					}
				}
				else
				{
					// Only for some tests...
					// when _virtualPlacePrior=0, set all priors to the same value
					if(cols>1)
					{
						float val = 1.0/cols;
						for(int j=0; j<cols; j++)
						{
							((float*)prediction.data)[i + j*cols] = val;
						}
					}
					else if(cols>0)
					{
						((float*)prediction.data)[i] = 1;
					}
				}
			}
		}
	}

	ULOGGER_DEBUG("time = %fs", timerGlobal.ticks());

	return prediction;
}

void BayesFilter::normalize(cv::Mat & prediction, unsigned int index, float addedProbabilitiesSum, bool virtualPlaceUsed) const
{
	UASSERT(index < (unsigned int)prediction.rows && index < (unsigned int)prediction.cols);

	int cols = prediction.cols;
	// ADD values of not found neighbors to loop closure
	if(addedProbabilitiesSum < _totalPredictionLCValues-_predictionLC[0])
	{
		float delta = _totalPredictionLCValues-_predictionLC[0]-addedProbabilitiesSum;
		((float*)prediction.data)[index + index*cols] += delta;
		addedProbabilitiesSum+=delta;
	}

	float allOtherPlacesValue = 0;
	if(_totalPredictionLCValues < 1)
	{
		allOtherPlacesValue = 1.0f - _totalPredictionLCValues;
	}

	// Set all loop events to small values according to the model
	if(allOtherPlacesValue > 0 && cols>1)
	{
		float value = allOtherPlacesValue / float(cols - 1);
		for(int j=virtualPlaceUsed?1:0; j<cols; ++j)
		{
			if(((float*)prediction.data)[index + j*cols] == 0)
			{
				((float*)prediction.data)[index + j*cols] = value;
				addedProbabilitiesSum += ((float*)prediction.data)[index + j*cols];
			}
		}
	}

	//normalize this row
	float maxNorm = 1 - (virtualPlaceUsed?_predictionLC[0]:0); // 1 - virtual place probability
	if(addedProbabilitiesSum<maxNorm-0.0001 || addedProbabilitiesSum>maxNorm+0.0001)
	{
		for(int j=virtualPlaceUsed?1:0; j<cols; ++j)
		{
			((float*)prediction.data)[index + j*cols] *= maxNorm / addedProbabilitiesSum;
		}
		addedProbabilitiesSum = maxNorm;
	}

	// ADD virtual place prob
	if(virtualPlaceUsed)
	{
		((float*)prediction.data)[index] = _predictionLC[0];
		addedProbabilitiesSum += ((float*)prediction.data)[index];
	}

	//debug
	//for(int j=0; j<cols; ++j)
	//{
	//	ULOGGER_DEBUG("test col=%d = %f", i, prediction.data.fl[i + j*cols]);
	//}

	if(addedProbabilitiesSum<0.99 || addedProbabilitiesSum > 1.01)
	{
		UWARN("Prediction is not normalized sum=%f", addedProbabilitiesSum);
	}
}

cv::Mat BayesFilter::updatePrediction(const cv::Mat & oldPrediction,
		const Memory * memory,
		const std::vector<int> & oldIds,
		const std::vector<int> & newIds) const
{
	UTimer timer;
	UDEBUG("");

	UASSERT(memory &&
		oldIds.size() &&
		newIds.size() &&
		oldIds.size() == (unsigned int)oldPrediction.cols &&
		oldIds.size() == (unsigned int)oldPrediction.rows);

	cv::Mat prediction = cv::Mat::zeros(newIds.size(), newIds.size(), CV_32FC1);

	// Create id to index maps
	std::map<int, int> oldIdToIndexMap;
	std::map<int, int> newIdToIndexMap;
	for(unsigned int i=0; i<oldIds.size() || i<newIds.size(); ++i)
	{
		if(i<oldIds.size())
		{
			UASSERT(oldIds[i]);
			oldIdToIndexMap.insert(oldIdToIndexMap.end(), std::make_pair(oldIds[i], i));
			//UDEBUG("oldIdToIndexMap[%d] = %d", oldIds[i], i);
		}
		if(i<newIds.size())
		{
			UASSERT(newIds[i]);
			newIdToIndexMap.insert(newIdToIndexMap.end(), std::make_pair(newIds[i], i));
			//UDEBUG("newIdToIndexMap[%d] = %d", newIds[i], i);
		}
	}
	UDEBUG("time creating id-index maps = %fs", timer.restart());

	//Get removed ids
	std::set<int> removedIds;
	for(unsigned int i=0; i<oldIds.size(); ++i)
	{
		if(!uContains(newIdToIndexMap, oldIds[i]))
		{
			removedIds.insert(removedIds.end(), oldIds[i]);
			UDEBUG("removed id=%d at oldIndex=%d", oldIds[i], i);
		}
	}
	UDEBUG("time getting removed ids = %fs", timer.restart());

	int added = 0;
	// get ids to update
	std::set<int> idsToUpdate;
	for(unsigned int i=0; i<oldIds.size() || i<newIds.size(); ++i)
	{
		if(i<oldIds.size())
		{
			if(removedIds.find(oldIds[i]) != removedIds.end())
			{
				unsigned int cols = oldPrediction.cols;
				for(unsigned int j=0; j<cols; ++j)
				{
					if(((const float *)oldPrediction.data)[i + j*cols] != 0.0f &&
					   j!=i &&
					   removedIds.find(oldIds[j]) == removedIds.end())
					{
						//UDEBUG("to update id=%d from id=%d removed (value=%f)", oldIds[j], oldIds[i], ((const float *)oldPrediction.data)[i + j*cols]);
						idsToUpdate.insert(oldIds[j]);
					}
				}
			}
		}
		if(i<newIds.size() && !uContains(oldIdToIndexMap,newIds[i]))
		{
			std::map<int, int> neighbors = memory->getNeighborsId(newIds[i], _predictionLC.size()-1, 0);
			float sum = this->addNeighborProb(prediction, i, neighbors, newIdToIndexMap);
			this->normalize(prediction, i, sum, newIds[0]<0);
			++added;
			for(std::map<int,int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				if(uContains(oldIdToIndexMap, iter->first) &&
				   removedIds.find(iter->first) == removedIds.end())
				{
					idsToUpdate.insert(iter->first);
				}
			}
		}
	}
	UDEBUG("time getting ids to update = %fs", timer.restart());

	// update modified/added ids
	int modified = 0;
	for(std::set<int>::iterator iter = idsToUpdate.begin(); iter!=idsToUpdate.end(); ++iter)
	{
		std::map<int, int> neighbors = memory->getNeighborsId(*iter, _predictionLC.size()-1, 0);
		int index = newIdToIndexMap.at(*iter);
		float sum = this->addNeighborProb(prediction, index, neighbors, newIdToIndexMap);
		this->normalize(prediction, index, sum, newIds[0]<0);
		++modified;
	}
	UDEBUG("time updating modified/added ids = %fs", timer.restart());

	//UDEBUG("oldIds.size()=%d, oldPrediction.cols=%d, oldPrediction.rows=%d", oldIds.size(), oldPrediction.cols, oldPrediction.rows);
	//UDEBUG("newIdToIndexMap.size()=%d, prediction.cols=%d, prediction.rows=%d", newIdToIndexMap.size(), prediction.cols, prediction.rows);
	// copy not changed probabilities
	int copied = 0;
	for(unsigned int i=0; i<oldIds.size(); ++i)
	{
		if(oldIds[i]>0 && removedIds.find(oldIds[i]) == removedIds.end() && idsToUpdate.find(oldIds[i]) == idsToUpdate.end())
		{
			for(int j=0; j<oldPrediction.cols; ++j)
			{
				if(removedIds.find(oldIds[j]) == removedIds.end() && ((const float *)oldPrediction.data)[i + j*oldPrediction.cols] != 0.0f)
				{
					//UDEBUG("i=%d, j=%d", i, j);
					//UDEBUG("oldIds[i]=%d, oldIds[j]=%d", oldIds[i], oldIds[j]);
					//UDEBUG("newIdToIndexMap.at(oldIds[i])=%d", newIdToIndexMap.at(oldIds[i]));
					//UDEBUG("newIdToIndexMap.at(oldIds[j])=%d", newIdToIndexMap.at(oldIds[j]));
					((float *)prediction.data)[newIdToIndexMap.at(oldIds[i]) + newIdToIndexMap.at(oldIds[j])*prediction.cols] = ((const float *)oldPrediction.data)[i + j*oldPrediction.cols];
				}
			}
			++copied;
		}
	}
	UDEBUG("time copying = %fs", timer.restart());

	//update virtual place
	if(newIds[0] < 0)
	{
		if(prediction.cols>1) // The first must be the virtual place
		{
			((float*)prediction.data)[0] = _virtualPlacePrior;
			float val = (1.0-_virtualPlacePrior)/(prediction.cols-1);
			for(int j=1; j<prediction.cols; j++)
			{
				((float*)prediction.data)[j*prediction.cols] = val;
			}
		}
		else if(prediction.cols>0)
		{
			((float*)prediction.data)[0] = 1;
		}
	}
	UDEBUG("time updating virtual place = %fs", timer.restart());

	UDEBUG("Modified=%d, Added=%d, Copied=%d", modified, added, copied);
	return prediction;
}

void BayesFilter::updatePosterior(const Memory * memory, const std::vector<int> & likelihoodIds)
{
	ULOGGER_DEBUG("");
	std::map<int, float> newPosterior;
	for(std::vector<int>::const_iterator i=likelihoodIds.begin(); i != likelihoodIds.end(); ++i)
	{
		std::map<int, float>::iterator post = _posterior.find(*i);
		if(post == _posterior.end())
		{
			if(_posterior.size() == 0)
			{
				newPosterior.insert(std::pair<int, float>(*i, 1));
			}
			else
			{
				newPosterior.insert(std::pair<int, float>(*i, 0));
			}
		}
		else
		{
			newPosterior.insert(std::pair<int, float>((*post).first, (*post).second));
		}
	}
	_posterior = newPosterior;
}

float BayesFilter::addNeighborProb(cv::Mat & prediction, unsigned int col, const std::map<int, int> & neighbors, const std::map<int, int> & idToIndexMap) const
{
	UASSERT((unsigned int)prediction.cols == idToIndexMap.size() &&
			(unsigned int)prediction.rows == idToIndexMap.size() &&
			col < (unsigned int)prediction.cols &&
			col < (unsigned int)prediction.rows);

	float sum=0;
	for(std::map<int, int>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		int index = uValue(idToIndexMap, iter->first, -1);
		if(index >= 0)
		{
			sum += ((float*)prediction.data)[col + index*prediction.cols] = _predictionLC[iter->second+1];
		}
	}
	return sum;
}


} // namespace rtabmap
