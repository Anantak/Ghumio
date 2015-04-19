/**
 *  FilterSettings
 *
 *  This represents settings for the filter. This object is created before the filter is created.
 *  Settings include a unique filter name, intended filtering time interval, frequency of state
 *  propagation, set of models used in the filter etc. The settings object is read from a protobuf
 *  file. Its main purpose is to encapsulate the protobuf message that provides filter settings.
 *
 *  There are doubts if this is really needed. This would probably have to reimplement all the
 *  accessors that protobuf already has. So may be no need for this class.
 *
 */

