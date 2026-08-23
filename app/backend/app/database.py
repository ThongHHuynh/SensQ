from sqlalchemy import select
from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine

from datetime import datetime

from .config import DATABASE_URL, DATA_DIR
from .models import Base, MissionRecord, RobotEvent, RobotSnapshot, SavedMap, Setting


engine = create_async_engine(DATABASE_URL, pool_pre_ping=True)
AsyncSessionLocal = async_sessionmaker(engine, expire_on_commit=False)


async def init_db() -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def save_event(source: str, message: str, level: str = "info") -> None:
    async with AsyncSessionLocal() as session:
        session.add(RobotEvent(source=source, message=message, level=level))
        await session.commit()


async def save_snapshot(payload: dict) -> None:
    async with AsyncSessionLocal() as session:
        session.add(RobotSnapshot(payload=payload))
        await session.commit()


async def create_saved_map(name: str, yaml_path: str, image_path: str, frame_id: str = "map", resolution: str = "Unknown") -> SavedMap:
    async with AsyncSessionLocal() as session:
        saved_map = SavedMap(name=name, yaml_path=yaml_path, image_path=image_path, frame_id=frame_id, resolution=resolution)
        session.add(saved_map)
        await session.commit()
        await session.refresh(saved_map)
        return saved_map


async def list_saved_maps() -> list[SavedMap]:
    async with AsyncSessionLocal() as session:
        result = await session.execute(select(SavedMap).order_by(SavedMap.created_at.desc()))
        return list(result.scalars().all())


async def get_saved_map(map_id: int) -> SavedMap | None:
    async with AsyncSessionLocal() as session:
        return await session.get(SavedMap, map_id)


async def rename_saved_map(map_id: int, name: str) -> SavedMap | None:
    async with AsyncSessionLocal() as session:
        saved_map = await session.get(SavedMap, map_id)
        if saved_map is None:
            return None

        saved_map.name = name
        await session.commit()
        await session.refresh(saved_map)
        return saved_map


async def create_mission_record(
    mission_type: str,
    dock_id: str,
    started_at: datetime,
    success: bool,
    area_covered_m2: float,
    duration_seconds: float,
    message: str,
) -> MissionRecord:
    async with AsyncSessionLocal() as session:
        record = MissionRecord(
            mission_type=mission_type,
            dock_id=dock_id,
            started_at=started_at,
            success=success,
            area_covered_m2=area_covered_m2,
            duration_seconds=duration_seconds,
            message=message,
        )
        session.add(record)
        await session.commit()
        await session.refresh(record)
        return record


async def list_mission_records(limit: int = 50) -> list[MissionRecord]:
    async with AsyncSessionLocal() as session:
        result = await session.execute(
            select(MissionRecord).order_by(MissionRecord.completed_at.desc()).limit(limit)
        )
        return list(result.scalars().all())


async def get_all_settings() -> dict:
    async with AsyncSessionLocal() as session:
        result = await session.execute(select(Setting))
        return {setting.key: setting.value for setting in result.scalars().all()}


async def upsert_settings(values: dict) -> dict:
    async with AsyncSessionLocal() as session:
        for key, value in values.items():
            existing = await session.execute(select(Setting).where(Setting.key == key))
            setting = existing.scalar_one_or_none()
            if setting is None:
                session.add(Setting(key=key, value=value))
            else:
                setting.value = value
        await session.commit()
        result = await session.execute(select(Setting))
        return {setting.key: setting.value for setting in result.scalars().all()}
