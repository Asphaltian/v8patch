#pragma once

#include <cstdint>
#include <string_view>

namespace v8patch::offsets {

struct Site
{
	std::uintptr_t offset;
	std::string_view signature;
	std::string_view name;
};

inline constexpr std::string_view kSehPrologue = "6A FF 68 ?? ?? ?? ?? 64 A1 00 00 00 00 50";

namespace RunService {

inline constexpr Site kConstructor{0x16ADD0, kSehPrologue, "RunService::RunService"};
inline constexpr Site kInvalidateRunViews{0x169800, kSehPrologue, "RunService::invalidateRunViews"};
inline constexpr Site kRunProc{
    0x1694E0,
    "6A FF 68 ?? ?? ?? ?? 64 A1 00 00 00 00 50 64 89 25 00 00 00 00 83 EC 74",
    "RunService::runProc",
};
inline constexpr Site kRaiseStepped{
    0x16B840,
    "83 EC 08 D9 44 24 0C 56 D9 5C 24 04 8B F1 D9 44 24 14",
    "RunService::raiseStepped",
};

inline constexpr std::uintptr_t kFramePeriod = 0x19C;

} // namespace RunService

namespace Security {

inline constexpr Site kDemand{
    0xA4DD0,
    "6A FF 68 ?? ?? ?? ?? 64 A1 00 00 00 00 50 81 EC 88 00 00 00 A1",
    "Security::demand",
};

} // namespace Security

namespace StandardOut {

inline constexpr Site kPrint{
    0x182C60,
    "6A FF 68 ?? ?? ?? ?? 64 A1 00 00 00 00 50 64 89 25 00 00 00 00 83 EC 24 8B 4C 24 3C",
    "StandardOut::print",
};

} // namespace StandardOut

namespace Constants {

inline constexpr Site kWorldStepsPerSec{0x2462C0, "B8 F0 00 00 00 C3", "Constants::worldStepsPerSec"};
inline constexpr Site kWorldDt{0x246300, "D9 05 ?? ?? ?? ?? C3", "Constants::worldDt"};

inline constexpr std::uintptr_t kImmediateOperand = 1;
inline constexpr std::uintptr_t kLoadOperand = 2;

} // namespace Constants

namespace World {

inline constexpr Site kStep{
    0x1EEA70,
    "6A FF 68 ?? ?? ?? ?? 64 A1 00 00 00 00 50 64 89 25 00 00 00 00 83 EC 78 A1 ?? ?? ?? ?? 53 56 8B F1",
    "World::step",
};

inline constexpr Site kRaiseAutoDestroy{
    0x1EE8D0,
    "83 EC 0C 56 8B F1 8B 56 08 33 C0 85 D2 57 89 44 24 08 75 04 33 C9 EB 08 8B 4E",
    "Notifier<World, AutoDestroy>::raise",
};

inline constexpr std::uintptr_t kAutoDestroyNotifier = 0x18;

inline constexpr Site kComputeFallen{
    0x1EE680,
    "55 8B EC 83 E4 F8 83 EC 24 53 56 8B 71 34 8B 06 8B 50 04 57 8B CE FF D2 83 F8",
    "World::computeFallen",
};

inline constexpr Site kRemovePrimitive{
    0x1EF2D0,
    "56 8B 74 24 08 57 6A 00 56 8B F9 E8 ?? ?? ?? ?? 8B 4F 30 56 E8 ?? ?? ?? ?? 8B 4F 34",
    "World::removePrimitive",
};

inline constexpr Site kOnPrimitiveTouched{
    0x1EE650,
    "56 8B F1 8D 44 24 08 50 8D 4E 38 E8 ?? ?? ?? ?? 8D 4C 24 0C 51 8D 4E 44",
    "World::onPrimitiveTouched",
};

} // namespace World

namespace Explosion {

inline constexpr Site kDoBlast{
    0x224EA0,
    "D9 EE 83 EC 48 57 8B F9 D8 9F 20 01 00 00 DF E0 F6 C4 05 0F 8A ?? ?? ?? ?? 53 55 56",
    "Explosion::doBlast",
};

inline constexpr std::uintptr_t kPosition = 0x110;
inline constexpr std::uintptr_t kBlastRadius = 0x11C;
inline constexpr std::uintptr_t kBlastPressure = 0x120;

} // namespace Explosion

namespace PartInstance {

inline constexpr Site kFromPrimitive{
    0x1AE5D0,
    "8B 44 24 04 85 C0 74 0D 8B 40 7C 85 C0 74 06 05 80 FE FF FF",
    "PartInstance::fromPrimitive",
};

inline constexpr Site kDestroyJoints{
    0x1AD9A0,
    "56 8B F1 56 E8 ?? ?? ?? ?? 83 C4 04 85 C0 74 0E 8B 8E E8 01",
    "PartInstance::destroyJoints",
};

} // namespace PartInstance

namespace ForceField {

inline constexpr Site kPartInForceField{
    0x2286E0,
    "56 8B 74 24 08 56 E8 ?? ?? ?? ?? 83 C4 04 84 C0 75 32 8B B6 CC 00 00 00",
    "ForceField::partInForceField",
};

} // namespace ForceField

namespace Array {

inline constexpr Site kResize{
    0x1ADF40,
    "8B 44 24 04 53 55 56 8B F1 8B 6E 04 B9 01 00 00 00 89 46 04 84 0D",
    "G3D::Array::resize",
};

} // namespace Array

namespace SpatialHash {

inline constexpr Site kPrimitiveExtentsChanged{
    0x21B4F0,
    "83 EC 60 53 55 33 C0 56 8B 74 24 70 57 8B F9 8B CE 89 44 24 10 89 44 24 14",
    "SpatialHash::primitiveExtentsChanged",
};

} // namespace SpatialHash

namespace IMoving {

inline constexpr Site kNotifyMoved{
    0x21F9E0,
    "83 EC 10 56 8B F1 83 7E 08 00 75 2C 8B 06 8B 10 6A 00 C7 46 08 1E 00 00",
    "IMoving::notifyMoved",
};

} // namespace IMoving

namespace Body {

inline constexpr Site kStateIndex{
    0x21F313,
    "A1 ?? ?? ?? ?? 83 C0 01 3D FF FF FF 7F A3 ?? ?? ?? ?? 75 0A B8 01 00 00 00",
    "Body::getNextStateIndex",
};

inline constexpr std::uintptr_t kStateIndexOperand = 1;

} // namespace Body

namespace Cofm {

inline constexpr Site kUpdate{
    0x261E40,
    "81 EC 88 00 00 00 53 8B D9 80 7B 04 00 0F 84 ?? ?? ?? ?? 55 56 8B 33",
    "Cofm::update",
};

} // namespace Cofm

namespace SimBody {

inline constexpr Site kUpdate{
    0x261930,
    "D9 EE 81 EC 80 00 00 00 53 8B D9 8B 03 8B 48 1C 85 C9 74 09 DD D8 E8",
    "SimBody::update",
};

inline constexpr Site kAccumulateForce{
    0x224CC0,
    "56 8B F1 80 7E 04 00 74 05 E8 ?? ?? ?? ?? 8B 44 24 08 D9 86 80 00 00 00",
    "SimBody::accumulateForce",
};

} // namespace SimBody

} // namespace v8patch::offsets
